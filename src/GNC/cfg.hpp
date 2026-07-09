#pragma once
#include <Eigen/Dense>

struct NAVc {
    double dt_s;
    Eigen::VectorXd x0;
};

struct CTLc {
    float angle_loop_dt_s;
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

struct GNCc {
    NAVc navc;
    CTLc ctlc;
    PID_3DOFc rate;
    PID_3DOFc angle;
    float dt_s;
};
