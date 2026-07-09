#include "quat_util.hpp"
#include <cmath>
#include <algorithm>

Eigen::Vector3f quat_to_euler(const Eigen::Quaternionf &q) {
    Eigen::Vector3f euler;

    // roll (x-axis rotation)
    float sinr_cosp = 2.0f * (q.w() * q.x() + q.y() * q.z());
    float cosr_cosp = 1.0f - 2.0f * (q.x() * q.x() + q.y() * q.y());
    euler.x() = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    float sinp = 2.0f * (q.w() * q.y() - q.z() * q.x());
    sinp = std::max(-1.0f, std::min(1.0f, sinp));
    euler.y() = std::asin(sinp);

    // yaw (z-axis rotation)
    float siny_cosp = 2.0f * (q.w() * q.z() + q.x() * q.y());
    float cosy_cosp = 1.0f - 2.0f * (q.y() * q.y() + q.z() * q.z());
    euler.z() = std::atan2(siny_cosp, cosy_cosp);

    return euler;
}
