/* https://opensource.org/licenses/Apache-2.0 */

#include "powermeterImpl.hpp"
#include <everest/logging.hpp>


namespace module {
namespace main {

void powermeterImpl::init() {
    energy_import_total = 0.0;
    energy_export_total = 0.0;

    power_supply_thread_handle = std::thread(&powermeterImpl::power_supply_worker, this);
}

void powermeterImpl::ready() {
    // sub mqtt topics with data provided by power supply dc
    mod->mqtt.subscribe("everest/simulation/power_supply_DC/voltage", [this](const std::string& data){
        this->voltage = std::stod(data);
    });
    mod->mqtt.subscribe("everest/simulation/power_supply_DC/current", [this](const std::string& data){
        this->current = std::stod(data);
    });
}

void powermeterImpl::power_supply_worker(void) {
    while (true) {
        if (power_supply_thread_handle.shouldExit()) {
            break;
        }

        // set interval for publishing
        std::this_thread::sleep_for(std::chrono::milliseconds(LOOP_SLEEP_MS));

        // publish power meter
        mod->p_main->publish_powermeter(power_meter_external(voltage, current));
    }
}

types::powermeter::Powermeter powermeterImpl::power_meter_external(double& voltage, double& current) {
    types::powermeter::Powermeter powermeter;

    powermeter.timestamp = Everest::Date::to_rfc3339(date::utc_clock::now());
    powermeter.meter_id = "DC_POWERMETER";

    if (current > 0) {
        energy_import_total += (voltage * current * LOOP_SLEEP_MS / 1000) / 3600;
    }
    if (current < 0) {
        energy_export_total += (voltage * -current * LOOP_SLEEP_MS / 1000) / 3600;
    }

    powermeter.energy_Wh_import = {static_cast<float>(energy_import_total)};
    powermeter.energy_Wh_export = {static_cast<float>(energy_export_total)};

    powermeter.power_W = {static_cast<float>(current * voltage)};
    powermeter.current_A = {static_cast<float>(current)};
    powermeter.voltage_V = {static_cast<float>(voltage)};

    return powermeter;
}

types::powermeter::TransactionStartResponse
powermeterImpl::handle_start_transaction(types::powermeter::TransactionReq& value) {
    // does nothing
    return {};
}

types::powermeter::TransactionStopResponse powermeterImpl::handle_stop_transaction(std::string& transaction_id) {
    // does nothing
    return {};
}

} // namespace main
} // namespace module
