#pragma once
#include "../bus.hpp"
#include "../cfg.hpp"
#include "PID/PID_3DOF.hpp"

class CTL {
public:
    CTL(GNCc cfg);
    CTLb update(const GNCb& gnc);

private:
    PID_3DOF rate_controller;
    PID_3DOF angle_controller;
    GNCc cfg_data;

    float time_accumulator_s;
    Eigen::Vector3f target_rates;
};
