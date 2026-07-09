#pragma once
#include <Eigen/Dense>

struct NAVc {
    double dt_s;
    Eigen::VectorXd x0;
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
    float angle_loop_dt_s;
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
    float dt_s;
};
