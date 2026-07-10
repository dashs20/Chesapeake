#pragma once
#include "../bus.hpp"
#include "../cfg.hpp"
#include "IMUfilter.h"

struct IMU_Compensated {
    Eigen::Vector3f accel_CG_mps2;
    Eigen::Vector3f omega_body_radps;
};

class NAV {
public:
    NAV(GNCc cfg);
    ~NAV();
    NAVb update(const GNCb& gnc);
    void reset();

private:
    double dt_s;
    GNCc cfg_data;
    IMUfilter* filter;

    IMU_Compensated compensate_imu(const GNCb& gnc);
};