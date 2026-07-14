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
    NAVb update(const ALLb& allb);
    void reset();
    CAL_FEEDBACKb get_cal_feedback() const { return cal_feedback; }

private:
    double dt_s;
    GNCc cfg_data;
    IMUfilter* filter;

    bool is_calibrating;
    int calibration_counter;
    float sum_ax;
    float sum_ay;
    float sum_az;
    float sum_gx;
    float sum_gy;
    float sum_gz;
    CAL_FEEDBACKb cal_feedback;

    IMU_Compensated compensate_imu(const ALLb& allb);
};