/* license */
#ifndef MAIN_POWER_SUPPLY_DC_IMPL_HPP
#define MAIN_POWER_SUPPLY_DC_IMPL_HPP

//
// AUTO GENERATED - MARKED REGIONS WILL BE KEPT
// template version 3
//

#include <generated/interfaces/power_supply_DC/Implementation.hpp>

#include "../CharxPSM2.hpp"

// ev@75ac1216-19eb-4182-a85c-820f1fc2c091:v1
// insert your custom include headers here
// ev@75ac1216-19eb-4182-a85c-820f1fc2c091:v1

namespace module {
namespace main {

struct Conf {};

class power_supply_DCImpl : public power_supply_DCImplBase {
public:
    power_supply_DCImpl() = delete;
    power_supply_DCImpl(Everest::ModuleAdapter* ev, const Everest::PtrContainer<CharxPSM2>& mod, Conf& config) :
        power_supply_DCImplBase(ev, "main"), mod(mod), config(config){};

    // ev@8ea32d28-373f-4c90-ae5e-b4fcc74e2a61:v1
    // insert your public definitions here
    // ev@8ea32d28-373f-4c90-ae5e-b4fcc74e2a61:v1

protected:
    // command handler functions (virtual)
    virtual void handle_setMode(types::power_supply_DC::Mode& mode, 
                                types::power_supply_DC::ChargingPhase& phase) override; 
    virtual void handle_setExportVoltageCurrent(double& voltage, double& current) override;
    virtual void handle_setImportVoltageCurrent(double& voltage, double& current) override;

    // ev@d2d1847a-7b88-41dd-ad07-92785f06f5c4:v1
    // insert your protected definitions here
    // ev@d2d1847a-7b88-41dd-ad07-92785f06f5c4:v1

private:
    const Everest::PtrContainer<CharxPSM2>& mod;
    const Conf& config;

    virtual void init() override;
    virtual void ready() override;

    // ev@3370e4dd-95f4-47a9-aaec-ea76f34a66c9:v1
    void system_broadcast_loop();
    void group_broadcast_loop();

    void handle_statuses(std::array<uint8_t, 5>& status_array);
    void handle_status2(uint8_t power_module_status);
    void handle_status1(uint8_t power_module_status);
    void handle_status0(uint8_t power_module_status);

    bool powermeter_simulated;

    float config_current_limit{0};
    float config_voltage_limit{0};
    float config_power_limit{0};
    float config_min_voltage_limit{50.};
    int config_power_modules_count;
    int config_broadcast_mode;
    int config_pwr_mdl_group_id;
    uint8_t config_power_modules_number;

    uint8_t active_number_of_pwr_mdls;

    std::atomic<float> voltage;
    std::atomic<float> current;

    std::array<uint8_t, 5> status_array;
    // ev@3370e4dd-95f4-47a9-aaec-ea76f34a66c9:v1
};

// ev@3d7da0ad-02c2-493d-9920-0bbbd56b9876:v1
// insert other definitions here
// ev@3d7da0ad-02c2-493d-9920-0bbbd56b9876:v1

} // namespace main
} // namespace module

#endif // MAIN_POWER_SUPPLY_DC_IMPL_HPP
