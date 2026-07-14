#pragma once
#include <Eigen/Dense>

struct IMUb {
    Eigen::Vector3f omega_body_radps;
    Eigen::Vector3f accel_body_mps2;
};

struct RCRXb {
    float arm_frac;
    float mode_frac;
    float thr_frac;
    float roll_frac;
    float pitch_frac;
    float yaw_frac;
};

struct HALb {
    IMUb imub;
    RCRXb rcrxb;
    float vbat_volts;
    float execution_time_ms;
};

struct CFG_APPb {
    bool calibrate_requested;
    bool reboot_requested;
    bool save_requested;
    bool defaults_requested;
    bool is_calibrating;
    float calibration_progress_frac;
};
