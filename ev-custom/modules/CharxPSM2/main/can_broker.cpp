#include "can_broker.hpp"

#include <cstring>
#include <stdexcept>

#include <net/if.h>
#include <poll.h>
#include <sys/eventfd.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <everest/logging.hpp>

namespace charx = can::protocol::charxpsm2;

// Helper function to throw an exception with an error message and errno description
static void throw_with_error(const std::string& msg) {
    throw std::runtime_error(msg + ": (" + std::string(strerror(errno)) + ")");
}

// Constructor for CanBroker: initializes the CAN socket and binds to the specified interface
CanBroker::CanBroker(const std::string& interface_name) {
    // Create a socket for CAN communication
    can_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);

    if (can_fd == -1) {
        throw_with_error("Failed to open socket");
    }

    // Retrieve interface index from interface name
    struct ifreq ifr;
    if (interface_name.size() >= sizeof(ifr.ifr_name)) {
        throw_with_error("Interface name too long: " + interface_name);
    } else {
        strcpy(ifr.ifr_name, interface_name.c_str());
    }
    if (ioctl(can_fd, SIOCGIFINDEX, &ifr) == -1) {
        throw_with_error("Failed with ioctl/SIOCGIFINDEX on interface " + interface_name);
    }

    // Bind the socket to the CAN interface
    struct sockaddr_can addr;
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(can_fd, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) == -1) {
        throw_with_error("Failed with bind");
    }

    // Create an event file descriptor for signaling thread termination
    event_fd = eventfd(0, 0);

    // Start the background loop thread
    loop_thread = std::thread(&CanBroker::loop, this);
}

// Destructor for CanBroker: cleans up resources and stops the loop thread
CanBroker::~CanBroker() {
    uint64_t quit_value = 1;
    write(event_fd, &quit_value, sizeof(quit_value)); // Signal the loop thread to exit
    loop_thread.join(); // Wait for the loop thread to finish
    close(can_fd);      // Close the CAN socket
    close(event_fd);    // Close the event file descriptor
}

// Listen for incoming CAN frames
void CanBroker::loop() {
    std::array<struct pollfd, 2> pollfds = {{
        {can_fd, POLLIN, 0},
        {event_fd, POLLIN, 0},
    }};

    while (true) {
        // test_end

        const auto poll_result = poll(pollfds.data(), pollfds.size(), -1);

        if (poll_result == 0) {
            // timeout
            continue;
        }

        if (pollfds[0].revents & POLLIN) {
            // frame handling
            struct can_frame frame;
            read(can_fd, &frame, sizeof(frame));
            handle_can_input(frame);
        }

        if (pollfds[1].revents & POLLIN) {
            // new event, for now, we do not care, later on we could check, if it is an exit event code
            uint64_t tmp;
            read(event_fd, &tmp, sizeof(tmp));
            return;
        }
    }
}

void CanBroker::handle_can_input(can_frame& frame) {
    std::unique_lock<std::mutex> request_lock(request.mutex);


    // Check if we wait for response and if this is the response we are waiting for
    if ((request.state != CanRequest::State::ISSUED) and (request.msg_type != get_msg_type(frame.can_id))) {
    return;
    }

    for (auto i = 0; i < request.response.size(); ++i) {
            request.response[i] = frame.data[i];
    }

    // Check identifier for error codes
    handle_errors(frame.can_id);

    request.state = CanRequest::State::COMPLETED;

    // Unlock for dispatch_frame to proceed
    request_lock.unlock();
    request.cv.notify_one();
}

void CanBroker::handle_errors(uint32_t can_id) {
    // delete extended frame flag
    can_id &= 0x1FFFFFFF;
    // get error_code
    uint8_t error_code = (can_id >> 26) & 0b111;
    // handle error code
    if (error_code == 0) {
        // normal code
        return;
    }
    if (error_code == 2) {
        // command invalid
        EVLOG_warning << "can response: command invalid";
        return;
    }
    if (error_code == 3) {
        // data invalid
        EVLOG_warning << "can response: data invalid";
        return;
    }
    if (error_code == 7) {
        // data invalid
        // EVLOG_warning << "can response: start of processing";
        return;
    }
}

// Try to establish connection, read number of power modules
void CanBroker::read_number_of_modules(bool& power_modules_connected, uint8_t& actual_number_of_pwr_mdls) {
    struct can_frame frame;
    std::vector<uint8_t> data(8, 0);
    uint64_t response;

    charx::prepare_frame(frame, monitor_id, broadcast_adr, charx::def::Command::SYSTEM_READ_MAX_VALUES, data); // Dunno why its called max values, returns number of power modules
    const auto status = dispatch_frame(frame, &response);


    if (status == CanBroker::AccessReturnType::SUCCESS) {
        response = __builtin_bswap64(response);

        EVLOG_info << std::hex << response;

        power_modules_connected = true;
        actual_number_of_pwr_mdls = (response >> 40) & 0xFF;
        EVLOG_info << "Power modules connected";
        EVLOG_info << "Number of connected power modules: " << static_cast<int>(actual_number_of_pwr_mdls);
    }
}

// Read individual power module statuses
CanBroker::AccessReturnType CanBroker::read_power_module_status(uint8_t module_address, std::array<uint8_t, 5>& status_list) {
    struct can_frame frame;
    std::vector<uint8_t> data(8, 0);
    uint64_t response;

    charx::prepare_frame(frame, monitor_id, module_address, 
                         charx::def::Command::MODULE_READ_STATUS, 
                         data);

    const auto status = dispatch_frame(frame, &response);

    if (status == CanBroker::AccessReturnType::SUCCESS) {
        EVLOG_info << "Power Module" << static_cast<int>(module_address) << " read sucesfully";
        charx::parse_statuses(status_list, response);
    }
    return status;
}

// Set system (broadcast) output voltage and current
CanBroker::AccessReturnType  CanBroker::set_system_voltage_current(const float& voltage, const float& current) {
    struct can_frame frame;
    std::vector<uint8_t> data(8, 0);
    uint64_t response;

    // Convert voltage and current to mV and mA
    uint32_t voltage_mV = static_cast<uint32_t>(voltage * 1000);
    uint32_t current_mA = static_cast<uint32_t>(current * 1000);

    // Set voltage bytes (0-3: MSB to LSB)
    data[0] = (voltage_mV >> 24) & 0xFF;
    data[1] = (voltage_mV >> 16) & 0xFF;
    data[2] = (voltage_mV >> 8) & 0xFF;
    data[3] = voltage_mV & 0xFF;

    // Set current bytes (4-7: MSB to LSB)
    data[4] = (current_mA >> 24) & 0xFF;
    data[5] = (current_mA >> 16) & 0xFF;
    data[6] = (current_mA >> 8) & 0xFF;
    data[7] = current_mA & 0xFF;


    // Prepare frame with broadcast addressing
    charx::prepare_frame(frame, monitor_id, broadcast_adr, 
                         charx::def::Command::SET_SYSTEM_OUTPUT_VOLTAGE_AND_CURRENT, 
                         data);

    const auto status = dispatch_frame(frame, &response);

    if (status == CanBroker::AccessReturnType::SUCCESS) {
        EVLOG_info << "System voltage and current set";
    }
    return status;
}

// read system (broadcast) voltage and current
CanBroker::AccessReturnType CanBroker::read_system_voltage_current(float& voltage, float& current) {
    struct can_frame frame;
    std::vector<uint8_t> data(8, 0);
    uint64_t response;

    // Prepare frame with broadcast addressing
    charx::prepare_frame(frame, monitor_id, broadcast_adr, 
                         charx::def::Command::SYSTEM_READ_ACTUAL_VALUES, 
                         data);

    const auto status = dispatch_frame(frame, &response);

    if (status == CanBroker::AccessReturnType::SUCCESS) {
        charx::parse_voltagecurrent(voltage, current, response);
    }
    return status;
}

// send frame, wait for response, adjust message status
CanBroker::AccessReturnType CanBroker::dispatch_frame(const can_frame& frame, uint64_t* response) {
    // Sends frame and waits for response
    std::lock_guard<std::mutex> access_lock(access_mtx);

    std::unique_lock<std::mutex> request_lock(request.mutex);

    // sends frame
    write_to_can(frame);

    request.id = invert_src_dst(frame.can_id);
    request.msg_type = get_msg_type(request.id);

    request.state = CanRequest::State::ISSUED;

    const auto finished = request.cv.wait_for(request_lock, ACCESS_TIMEOUT,
                                              [this]() { return request.state != CanRequest::State::ISSUED; });
                                            
    if (not finished) {
        EVLOG_info << "TIMEOUT";
        return AccessReturnType::TIMEOUT;
    }

    if (request.state == CanRequest::State::FAILED) {
        return AccessReturnType::FAILED;
    }

    // success
    if (response) {
        memcpy(response, request.response.data(), sizeof(std::remove_pointer_t<decltype(response)>));
    }

    return AccessReturnType::SUCCESS;
}

// invert source and destination adresses, use for response identification
uint32_t CanBroker::invert_src_dst(uint32_t can_id) {
    uint32_t src = can_id & 0x000000FF;
    uint32_t dst = (can_id & 0x0000FF00) >> 8;

    uint32_t swapped_bits = (src << 8) | dst;

    can_id &= 0xFFFF0000;
    can_id |= swapped_bits;
    return can_id;
}

// get only message type
uint32_t CanBroker::get_msg_type(uint32_t identifier) {
    uint32_t msg_type = identifier << 3; 
    return msg_type;
}

// Set the operational readiness of the device (enabled or disabled)
void CanBroker::set_state(bool enabled) {
    struct can_frame frame;
    std::vector<uint8_t> data(8, 0);
    uint64_t response;

    if (enabled == true) {
        EVLOG_info << "setting power modules on";
        data[0] = 0x00;
    }
    else {
        EVLOG_info << "setting power modules off";
        data[0] = 0x01; 
    }

    charx::prepare_frame(frame, monitor_id, broadcast_adr, charx::def::Command::SWITCH_OPERATIONAL_READINESS, data); // Prepares a frame for the command
    
    const auto status = dispatch_frame(frame, &response);

    if (status == CanBroker::AccessReturnType::SUCCESS) {
    }
}

// Write a CAN frame to the socket
void CanBroker::write_to_can(const struct can_frame& frame) {
    write(can_fd, &frame, sizeof(frame)); // Send the frame to the CAN bus
}