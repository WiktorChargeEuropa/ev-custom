/* license */
#include <memory>
#include <fmt/core.h>
#include <utils/formatter.hpp>
#include "power_supply_DCImpl.hpp"
#include <everest/logging.hpp>
#include "can_broker.hpp"

namespace charx = can::protocol::charxpsm2;

namespace module {
namespace main {

std::unique_ptr<CanBroker> can_broker;
bool power_modules_state;

static void log_status_on_fail(const std::string& msg, CanBroker::AccessReturnType status) {
    // CAN request returns status, this function logs failed request
    using ReturnStatus = CanBroker::AccessReturnType;

    std::string reason;
    switch (status) {
    case ReturnStatus::FAILED:
        reason = "failed";
        EVLOG_warning << msg << " reason: (" << reason << ")";
        break;
    case ReturnStatus::NOT_READY:
        reason = "not ready";
        EVLOG_warning << msg << " reason: (" << reason << ")";
        break;
    case ReturnStatus::TIMEOUT:
        reason = "timeout";
        EVLOG_warning << msg << " reason: (" << reason << ")";
        break;
    default:
        return;
    }
}

void power_supply_DCImpl::init() {
    current = 0;
    voltage = 0;

    config_broadcast_mode=mod->config.broadcast_mode;
    config_power_modules_number=mod->config.number_of_power_modules;
    config_pwr_mdl_group_id=mod->config.power_module_group_id;
    config_current_limit=mod->config.current_limit_A;
    config_voltage_limit=mod->config.voltage_limit_V;
    config_power_limit=mod->config.power_limit_W;
    
    can_broker = std::make_unique<CanBroker>(mod->config.device);
}

void power_supply_DCImpl::ready() {
    EVLOG_info << "implementation ready";

    // set capabilites
    types::power_supply_DC::Capabilities caps;
    caps.bidirectional = false;
    caps.max_export_current_A = config_current_limit;
    caps.max_export_voltage_V = config_voltage_limit;
    caps.min_export_current_A = 0;
    caps.min_export_voltage_V = config_min_voltage_limit;
    caps.max_export_power_W = config_power_limit;

    // publish capabilities
    publish_capabilities(caps);

    // ensure power modules operational status is off
    can_broker->set_state(false);

    // loop selection
    if (config_broadcast_mode == 1) {
        system_broadcast_loop();
    } else group_broadcast_loop();
}

void power_supply_DCImpl::system_broadcast_loop() {

    // set connection flag
    bool power_modules_connected = false;

    powermeter_simulated = true;

    while (true) {
        // the interval time for control requests should be between 50 ms and 200 ms.
        std::this_thread::sleep_for(std::chrono::milliseconds(125));

        // try to connect, read number of power modules in the system
        EVLOG_info << "Trying to read number of modules";
        can_broker->read_number_of_modules(power_modules_connected, active_number_of_pwr_mdls);

        // continue only if pwr mdls are connected and number of them is equal to an expected nmbr
        if ((power_modules_connected == true) && (active_number_of_pwr_mdls == config_power_modules_number)) {

                // set state
                can_broker->set_state(power_modules_state);

                // set voltage and current
                auto status = can_broker->set_system_voltage_current(voltage, current); // on broadcast mode, no response expected

                // read voltage and current, publish them
                EVLOG_info << "Reading system voltage and current";
                float tmp_voltage, tmp_current;
                types::power_supply_DC::VoltageCurrent vc;
                status = can_broker->read_system_voltage_current(tmp_voltage, tmp_current);
                log_status_on_fail("Reading system (voltage, current) error", status);
                
                // real values
                vc.voltage_V = tmp_voltage;
                vc.current_A = tmp_current;
                // simulation only 
                /*
                if (power_modules_state) {
                    vc.voltage_V = voltage;
                    vc.current_A = current;
                }
                else
                {
                    vc.voltage_V = 0.0;
                    vc.current_A = 0.0;
                } */

                EVLOG_info << "voltage: " << tmp_voltage << "current: " << tmp_current;
                publish_voltage_current(vc);

                // read individual power modules statuses
                for (uint8_t module_address = 0x00; module_address < config_power_modules_number; module_address++){
                    EVLOG_info << "read power module " << static_cast<int>(module_address);
                    auto status = can_broker->read_power_module_status(module_address, status_array);
                    std::string message = "Error reading status of power module 0x0" + std::to_string(module_address);
                    log_status_on_fail(message, status);
                    handle_statuses(status_array);
                }

                // powermeter simulation
                if (powermeter_simulated == true) {
                    if (power_modules_state) {
                        mod->mqtt.publish("everest/simulation/power_supply_DC/voltage", "\"" + std::to_string(voltage) + "\"");
                        mod->mqtt.publish("everest/simulation/power_supply_DC/current", "\"" + std::to_string(current) + "\"");
                    }
                    else
                    {
                        mod->mqtt.publish("everest/simulation/power_supply_DC/voltage", "\"0.0\"");
                        mod->mqtt.publish("everest/simulation/power_supply_DC/current", "\"0.0\"");
                    }
                }
        }
    }
}

void power_supply_DCImpl::group_broadcast_loop() {

    // set connection flag
    bool power_modules_connected = false;

    // try to connect, read number of power modules in the system
    EVLOG_info << "Trying to read number of modules";
    can_broker->read_number_of_modules(power_modules_connected, active_number_of_pwr_mdls);

    // to do
}

void power_supply_DCImpl::handle_setExportVoltageCurrent(double& voltage, double& current) {
    if (voltage <= config_voltage_limit && voltage >= config_min_voltage_limit && current <= config_current_limit) {
        EVLOG_info << "EXPORT---";
        this->voltage = voltage;
        this->current = current;
    } else {
        EVLOG_error << fmt::format("Out of range voltage/current settings ignored: {}V / {}A", voltage, current);
    }
}

void power_supply_DCImpl::handle_setMode(types::power_supply_DC::Mode& mode,
                                         types::power_supply_DC::ChargingPhase& phase) {
    if (mode == types::power_supply_DC::Mode::Export) {
        power_modules_state = true;
    } else {
        power_modules_state = false;
    }
}

void power_supply_DCImpl::handle_setImportVoltageCurrent(double& voltage, double& current) {
    // doesn't do anything
}

void power_supply_DCImpl::handle_statuses(std::array<uint8_t, 5>& status_array) {
    uint8_t power_module_group = status_array[0];
    uint8_t power_module_temp = status_array[1];
    
    uint8_t power_module_status2 = status_array[2];
    uint8_t power_module_status1 = status_array[3];
    uint8_t power_module_status0 = status_array[4];

    handle_status2(power_module_status2);
    handle_status1(power_module_status1);
    handle_status0(power_module_status0);
}

void power_supply_DCImpl::handle_status2(uint8_t power_module_status) {
    if (power_module_status & 1 == 1) {
        EVLOG_warning << "Output power limitation";
    }
    if (power_module_status & 2 == 1) {
        EVLOG_warning << "Module ID repetition";
    }
    if (power_module_status & 3 == 1) {
        EVLOG_warning << "Load sharing";
    }
    if (power_module_status & 4 == 1) {
        EVLOG_warning << "Input phase lost";
    }
    if (power_module_status & 5 == 1) {
        EVLOG_warning << "Input asymmetry";
    }
    if (power_module_status & 6 == 1) {
        EVLOG_warning << "Undervoltage at the input";
    }
    if (power_module_status & 7 == 1) {
        EVLOG_warning << "OOvervoltage at the input";
    }
    if (power_module_status & 8 == 1) {
        EVLOG_warning << "PFC circuit is OFF";
    }
}

void power_supply_DCImpl::handle_status1(uint8_t power_module_status) {
    if (power_module_status & 1 == 1) {
        EVLOG_warning << "DC side is OFF";
    }
    if (power_module_status & 2 == 1) {
        EVLOG_warning << "Module error";
    }
    if (power_module_status & 3 == 1) {
        EVLOG_warning << "Module protection";
    }
    if (power_module_status & 4 == 1) {
        EVLOG_warning << "Fan error";
    }
    if (power_module_status & 5 == 1) {
        EVLOG_warning << "Temperature threshold value exceeded";
    }
    if (power_module_status & 6 == 1) {
        EVLOG_warning << "Overvoltage at the output";
    }
    if (power_module_status & 7 == 1) {
        EVLOG_warning << "Slow startup ";
    }
    if (power_module_status & 8 == 1) {
        EVLOG_warning << "CAN command interruption";
    }
}

void power_supply_DCImpl::handle_status0(uint8_t power_module_status) {
    if (power_module_status & 1 == 1) {
        EVLOG_warning << "Output short circuit ";
    }
    if (power_module_status & 3 == 1) {
        EVLOG_warning << "Internal communication interruption ";
    }
    if (power_module_status & 4 == 1) {
        EVLOG_warning << "PFC circuit abnormal";
    }
    if (power_module_status & 6 == 1) {
        EVLOG_warning << "Discharge abnormal";
    }
}

} // namespace main
} // namespace module
