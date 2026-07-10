#pragma once
#include "../bus.hpp"
#include "../cfg.hpp"

struct IMU_Compensated {
    Eigen::Vector3f accel_CG_mps2;
    Eigen::Vector3f omega_body_radps;
};

class NAV {
public:
    NAV(GNCc cfg);
    NAVb update(const GNCb& gnc);

private:
    GNCc cfg_data;

    IMU_Compensated compensate_imu(const GNCb& gnc);
};