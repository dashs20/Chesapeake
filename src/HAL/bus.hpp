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
    float time_imu_us;
    float time_rcrx_us;
    float time_motors_us;
    float time_servos_us;
};

struct ACTb {
    float m1_frac;
    float m2_frac;
    float m3_frac;
    float m4_frac;
    float s1_deg;
    float s2_deg;
    float s3_deg;
    float s4_deg;
    float LED_blink_Hz;
};
