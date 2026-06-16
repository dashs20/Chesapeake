#pragma once
#include "allocation/alloc.hpp"
#include "controllers/pid.hpp"
#include "filters/lowpass_filter.hpp"
#include "util/util.hpp"

// Plain old data object; vehicle parameters
struct gnc_cfg {
  pid_cfg pid_x_cfg;           // Roll controller cfg
  pid_cfg pid_y_cfg;           // Pitch controller cfg
  pid_cfg pid_z_cfg;           // Yaw controller cfg
  alloc_cfg veh_alloc_cfg;     // Allocator config (actuator limits)
  lpf_cfg imu_lpf_cfg;         // IMU lpf config
  util::vec imu_euler_xyz_deg; // IMU orientation (euler XYZ, sequential)
};

class gnc {
public:
  gnc(gnc_cfg config); // Default Constructor
  ~gnc();              // Destructor
  act_cmd query(util::vec imu_raw_degps, util::vec rate_cmd_radps,
                double throttle); // Query member function

private:
  gnc_cfg config;
  lpf imu_x_lpf;   // IMU x lpf
  lpf imu_y_lpf;   // IMU y lpf
  lpf imu_z_lpf;   // IMU z lpf
  pid pid_x;       // Roll PID controller
  pid pid_y;       // Pitch PID controller
  pid pid_z;       // Yaw PID controller
  alloc allocator; // Actuator allocator
};