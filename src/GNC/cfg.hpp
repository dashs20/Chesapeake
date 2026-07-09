#pragma once
#include <Eigen/Dense>

struct NAVc {
    double dt_s;
    Eigen::VectorXd x0;
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

struct GNCc {
    NAVc navc;
    CTLc ctlc;
    GUIc guic;
    ALLOCc allocc;
    ALLOCATOR allocator;
    float dt_s;
};
