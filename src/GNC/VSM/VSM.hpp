#pragma once
#include "../bus.hpp"
#include "../cfg.hpp"
#include "../UTIL/StateMachine.hpp"

class VSM {
public:
    VSM(GNCc cfg);
    GNCb update(GNCb gnc);

private:
    GNCc cfg_data;
    StateMachine<STATE, GNCb> vehicle_mode_machine;
    StateMachine<ATT_MODE, GNCb> attitude_mode_machine;

    void setup_vehicle_mode_machine();
    void setup_attitude_mode_machine();
};
