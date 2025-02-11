#include "charxpsm2_protocol.hpp"

#include <algorithm>
#include <cstring>
#include <type_traits>

#include <endian.h>
#include <everest/logging.hpp>


namespace can::protocol::charxpsm2 
{
void prepare_frame(can_frame& frame, uint8_t source, uint8_t destination, def::Command commandNo, const std::vector<uint8_t>& input_data) {
    clear_frame(frame);
    
    // set identifier
    set_header(frame, source, destination, commandNo);

    constexpr std::size_t MAX_PAYLOAD_SIZE = 8;
    const auto payload_size = std::min(input_data.size(), MAX_PAYLOAD_SIZE);

    std::memcpy(frame.data, input_data.data(), payload_size);
    frame.can_dlc = payload_size;

    std::memcpy(frame.data, input_data.data(), std::min(input_data.size(), size_t(8)));
};

void set_header(struct can_frame& frame, uint8_t source, uint8_t destination, def::Command commandNo) {
    def::ErrorCode errorCode = def::ErrorCode::NORMAL;
    const uint8_t deviceNo = 0x0A;

    frame.can_id = (static_cast<uint8_t>(errorCode) << def::ERROR_CODE_BIT_SHIFT) |
         (deviceNo << def::DEVICE_NO_BIT_SHIFT) |
         (static_cast<uint8_t>(commandNo) << def::COMMAND_NO_BIT_SHIFT) |
         (destination << def::TARGET_ADDR_BIT_SHIFT) |
         (source << def::SOURCE_ADDR_BIT_SHIFT) | CAN_EFF_FLAG;                                                                                               
}

void set_data(struct can_frame& frame, bool switch_on, bool close_input_relay){
    frame.data[0] = 0x00;
    frame.data[2] |= 0x00;
    frame.data[2] |= 0x00;
    frame.data[3] = 0x80;
    frame.can_dlc = sizeof(frame.data);
}

void parse_voltagecurrent(float& voltage, float& current, uint64_t& response) {
    response = __builtin_bswap64(response);
    // first 4 bytes is voltage
    uint32_t current_raw = static_cast<uint32_t>(response & 0xFFFFFFFF);
    std::memcpy(&current, &current_raw, sizeof(float));
    // next 4 bytes is current
    uint32_t voltage_raw = static_cast<uint32_t>((response >> 32) & 0xFFFFFFFF);
    std::memcpy(&voltage, &voltage_raw, sizeof(float));
}

void parse_statuses(std::array<uint8_t, 5>& status_list, uint64_t& response) {
    response = __builtin_bswap64(response);
    status_list[0] = (response >> 40) & 0xFF;    // module group
    status_list[1] = (response >> 24) & 0xFF ;   // module temperature
    status_list[2] = (response >> 16) & 0xFF ;   // status 2
    status_list[4] = (response >> 8) & 0xFF ;    // status 0
    status_list[3] = response & 0xFF ;           // status 1
}

void clear_frame(can_frame& frame) {
    memset(frame.data, 0, sizeof(frame.data));
}

namespace prepare_data_for_command {
void read_actual_values(struct can_frame& frame) {
        for (int i = 0; i < 7; i++) {
            frame.data[i] = 0x00;
        }
        frame.can_dlc = 8;
    }
}
} // can::protocol::charxpsm2