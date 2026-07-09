#pragma once
#include "../bus.hpp"
#include "../cfg.hpp"
#include "UKF/UKF.hpp"

struct IMU_Compensated {
    Eigen::Vector3f accel_CG_mps2;
    Eigen::Vector3f omega_body_radps;
};

class NAV {
public:
    NAV(GNCc cfg);
    NAVb update(const GNCb& gnc);

private:
    double dt_s;
    Eigen::VectorXd x;
    UKF ukf;
    GNCc cfg_data;
    Eigen::Vector3f prev_omega_body_radps;

    // Private calibration & offset helper
    IMU_Compensated compensate_imu(const GNCb& gnc);
};