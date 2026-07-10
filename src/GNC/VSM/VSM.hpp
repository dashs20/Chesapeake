#pragma once
#include "../bus.hpp"
#include "../cfg.hpp"
#include "../UTIL/StateMachine.hpp"

class VSM {
public:
    VSM(GNCc cfg);
    VSMb update(const ALLb& allb);

private:
    GNCc cfg_data;
    StateMachine<STATE, ALLb> vehicle_mode_machine;
    StateMachine<ATT_MODE, ALLb> attitude_mode_machine;

    void setup_vehicle_mode_machine();
    void setup_attitude_mode_machine();
};
