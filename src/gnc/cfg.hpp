#pragma once
#include "controllers/pid.hpp"
#include "allocation/alloc.hpp"
#include "filters/lowpass_filter.hpp"
#include "util/util.hpp"
#include <string>

// Plain old data object; vehicle parameters
struct gnc_cfg {
  pid_cfg pid_x_cfg;           // Roll controller cfg
  pid_cfg pid_y_cfg;           // Pitch controller cfg
  pid_cfg pid_z_cfg;           // Yaw controller cfg
  alloc_cfg veh_alloc_cfg;     // Allocator config (actuator limits)
  lpf_cfg imu_lpf_cfg;         // IMU lpf config
  util::vec imu_euler_xyz_deg; // IMU orientation (euler XYZ, sequential)
};

// Loader function
gnc_cfg load_gnc_cfg(const std::string &filepath);
