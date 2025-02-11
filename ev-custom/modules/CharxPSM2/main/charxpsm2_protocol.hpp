#ifndef CAN_PROTOCOL_DPM1000_HPP
#define CAN_PROTOCOL_DPM1000_HPP

#include <cstdint>
#include <vector>
#include <array>

#include <linux/can.h>

namespace can::protocol::charxpsm2 {
namespace def {

enum class ErrorCode : uint8_t {
    NORMAL = 0x00,
    COMMAND_INVALID = 0x02,
    DATA_INVALID = 0x03,
    START_OF_PROCESSING = 0x07
};

enum class Command : uint8_t {
    SYSTEM_READ_ACTUAL_VALUES = 0x01,
    SYSTEM_READ_MAX_VALUES = 0x02,
    MODULE_READ_ACTUAL_VALUES = 0x03,
    MODULE_READ_STATUS = 0x04,
    READ_AC_INPUT_VOLTAGE = 0x06,
    MODULES_SLOW_STARTUP = 0x13,
    LED_CONTROL_GREEN = 0x14,
    READ_MODULE_INFO = 0x0C,
    LIMIT_INPUT_CURRENT = 0x0F,
    SWITCH_OPERATIONAL_READINESS = 0x1A,
    SET_SYSTEM_OUTPUT_VOLTAGE_AND_CURRENT = 0x1B,
    SET_MODULE_OUTPUT_VOLTAGE_AND_CURRENT = 0x1C
};

constexpr auto ERROR_CODE_BIT_SHIFT = 26;             // Bit shift for error code (bits 28-26)
constexpr auto DEVICE_NO_BIT_SHIFT = 22;              // Bit shift for device number (bits 25-22)
constexpr auto COMMAND_NO_BIT_SHIFT = 16;             // Bit shift for command number (bits 21-16)
constexpr auto TARGET_ADDR_BIT_SHIFT = 8;             // Bit shift for target address (bits 15-8)
constexpr auto SOURCE_ADDR_BIT_SHIFT = 0;             // Bit shift for source address (bits 7-0)
};

void prepare_frame(struct can_frame& frame, uint8_t source, uint8_t destination, def::Command commandNo, const std::vector<uint8_t>& data);
void set_data(struct can_frame& frame, bool switch_on, bool close_input_relay);
void set_header(struct can_frame& frame, uint8_t source, uint8_t destination, def::Command commandNo);
void clear_frame(can_frame& frame);
void parse_voltagecurrent(float& voltage, float& current, uint64_t& response);
void parse_statuses(std::array<uint8_t, 5>& status_list, uint64_t& response);
}

#endif