#pragma once
#include <Eigen/Dense>
#include "../../cfg.hpp"

class PID_scalar {
public:
    PID_scalar(PID_SCALARc config);
    float update(float setpoint, float measurement);
    void reset();

private:
    PID_SCALARc cfg;
    float integral;
    float prev_error;
};

class PID_3DOF {
public:
    PID_3DOF(PID_3DOFc config);
    Eigen::Vector3f update(Eigen::Vector3f setpoint, Eigen::Vector3f measurement);
    void reset();

    PID_scalar roll;
    PID_scalar pitch;
    PID_scalar yaw;
};
