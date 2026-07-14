#include "PID_3DOF.hpp"

PID_scalar::PID_scalar(PID_SCALARc config) : cfg(config), integral(0.0f), prev_error(0.0f) {}

void PID_scalar::reset() {
    integral = 0.0f;
    prev_error = 0.0f;
}

float PID_scalar::update(float setpoint, float measurement) {
    float error = (setpoint - measurement) * cfg.dt_s;

    integral += error * cfg.dt_s;
    float clamp_val = cfg.i_max * cfg.dt_s;
    if (integral > clamp_val) {
        integral = clamp_val;
    } else if (integral < -clamp_val) {
        integral = -clamp_val;
    }

    float derivative = 0.0f;
    if (cfg.dt_s > 0.0f) {
        derivative = (error - prev_error) / cfg.dt_s;
    }
    prev_error = error;

    float output = (cfg.kp * error) + (cfg.ki * integral) + (cfg.kd * derivative);

    if (output > cfg.out_max) {
        output = cfg.out_max;
    } else if (output < cfg.out_min) {
        output = cfg.out_min;
    }

    return output;
}

PID_3DOF::PID_3DOF(PID_3DOFc config) : roll(config.roll), pitch(config.pitch), yaw(config.yaw) {}

void PID_3DOF::reset() {
    roll.reset();
    pitch.reset();
    yaw.reset();
}

Eigen::Vector3f PID_3DOF::update(Eigen::Vector3f setpoint, Eigen::Vector3f measurement) {
    Eigen::Vector3f output;
    output.x() = roll.update(setpoint.x(), measurement.x());
    output.y() = pitch.update(setpoint.y(), measurement.y());
    output.z() = yaw.update(setpoint.z(), measurement.z());
    return output;
}
