#ifndef Charx_PSM2_CAN_BROKER_HPP
#define Charx_PSM2_CAN_BROKER_HPP

#include <array>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

#include"charxpsm2_protocol.hpp"

struct CanRequest {
    enum class State {
        IDLE,
        ISSUED,
        COMPLETED,
        FAILED,
    } state{State::IDLE};

    uint32_t id; // identifier
    uint32_t msg_type; // identifier but without error code
    std::array<uint8_t, 8> response; // frame data
    std::condition_variable cv;
    std::mutex mutex;
};

class CanBroker {
public:
    enum class AccessReturnType {
        SUCCESS,
        FAILED,
        TIMEOUT,
        NOT_READY,
    };

    CanBroker(const std::string& interface_name);

    void set_state(bool enabled);
    void read_number_of_modules(bool& power_modules_connected, uint8_t& actual_number_of_pwr_mdls);
    CanBroker::AccessReturnType set_system_voltage_current(const float& voltage, const float& current);
    CanBroker::AccessReturnType read_system_voltage_current(float& voltage, float& current);
    CanBroker::AccessReturnType read_power_module_status(uint8_t module_address, std::array<uint8_t, 5>& status_list);

    ~CanBroker();

private:
    constexpr static auto ACCESS_TIMEOUT = std::chrono::milliseconds(200);

    void loop();
    void write_to_can(const struct can_frame& frame);
    AccessReturnType dispatch_frame(const struct can_frame& frame, uint64_t* response = nullptr);
    uint32_t invert_src_dst(uint32_t can_id);
    void handle_can_input(can_frame& frame);
    uint32_t get_msg_type(uint32_t identifier);
    void handle_errors(uint32_t can_id);

    uint8_t device_src;
    uint8_t broadcast_adr{0x3F};
    std::mutex access_mtx;
    CanRequest request;
    const uint8_t monitor_id{0xf0};
    std::thread loop_thread;
    int event_fd{-1};
    int can_fd{-1};
};

#endif