#pragma once
#include <Eigen/Dense>

Eigen::Vector3f quat_to_euler(const Eigen::Quaternionf &q);
