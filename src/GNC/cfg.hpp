#pragma once
#include <Eigen/Dense>
#include <cstdint>

struct NAVc {
    Eigen::VectorXd x0;
    Eigen::Vector3f r_IMU2CG_mm;       // IMU position relative to CG in millimeters
    Eigen::Quaternionf q_IMU2body;      // Rotation from IMU sensor frame to vehicle body frame
};

struct PID_SCALARc {
    float kp;
    float ki;
    float kd;
    float i_max;
    float out_min;
    float out_max;
    float dt_s;
};

struct PID_3DOFc {
    PID_SCALARc roll;
    PID_SCALARc pitch;
    PID_SCALARc yaw;
};

struct CTLc {
    uint32_t angle_loop_hz;
    PID_3DOFc rate;
    PID_3DOFc angle;
};

enum class ALLOCATOR {
    QUAD
};

struct ALLOCc {
    float min_motor_frac;
    float servo_min_ang_deg;
    float servo_max_ang_deg;
    float servo_default_ang_deg;
    float blink_hz_disarmed;
    float blink_hz_rate;
    float blink_hz_angle;
    float blink_hz_gps_hold;
};

struct EXPOc {
    float roll;
    float pitch;
    float yaw;
};

struct GUIc {
    float max_rate_radps;
    float max_angle_rad;
    EXPOc expoc;
};

struct VSMc {
    float arm_threshold_frac;
    float mode_rate_threshold_frac;
    float mode_angle_threshold_frac;
};

struct GNCc {
    NAVc navc;
    CTLc ctlc;
    GUIc guic;
    ALLOCc allocc;
    VSMc vsmc;
    ALLOCATOR allocator;
    uint32_t looprate_hz;
};
