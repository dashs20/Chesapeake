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

struct MOTb {
    float m1_rpm;
    float m2_rpm;
    float m3_rpm;
    float m4_rpm;
};

struct HALb {
    IMUb imub;
    RCRXb rcrxb;
    MOTb motb;
    float vbat_volts;
};
