#include "cfg.hpp"
#include "util/util.hpp"

gnc_cfg load_gnc_cfg(const std::string &filepath) {
  util::config_parser parser;
  parser.load_file(filepath);

  gnc_cfg config;

  // GLOBAL
  double global_looprate = parser.get("looprate_hz");

  // ROLL PID
  config.pid_x_cfg.kp = parser.get("pid_x_kp");
  config.pid_x_cfg.ki = parser.get("pid_x_ki");
  config.pid_x_cfg.kd = parser.get("pid_x_kd");
  config.pid_x_cfg.i_lim = parser.get("pid_x_i_lim");
  config.pid_x_cfg.looprate_hz = global_looprate;

  // PITCH PID
  config.pid_y_cfg.kp = parser.get("pid_y_kp");
  config.pid_y_cfg.ki = parser.get("pid_y_ki");
  config.pid_y_cfg.kd = parser.get("pid_y_kd");
  config.pid_y_cfg.i_lim = parser.get("pid_y_i_lim");
  config.pid_y_cfg.looprate_hz = global_looprate;

  // YAW PID
  config.pid_z_cfg.kp = parser.get("pid_z_kp");
  config.pid_z_cfg.ki = parser.get("pid_z_ki");
  config.pid_z_cfg.kd = parser.get("pid_z_kd");
  config.pid_z_cfg.i_lim = parser.get("pid_z_i_lim");
  config.pid_z_cfg.looprate_hz = global_looprate;

  // ALLOCATOR
  config.veh_alloc_cfg.t_min_frac = parser.get("alloc_t_min_frac");
  config.veh_alloc_cfg.theta_min_deg = parser.get("alloc_theta_min_deg");
  config.veh_alloc_cfg.theta_max_deg = parser.get("alloc_theta_max_deg");

  // IMU LPF
  config.imu_lpf_cfg.fc_hz = parser.get("imu_lpf_fc_hz");
  config.imu_lpf_cfg.looprate_hz = global_looprate;

  // IMU ORIENTATION
  config.imu_euler_xyz_deg.x = parser.get("imu_euler_x_deg");
  config.imu_euler_xyz_deg.y = parser.get("imu_euler_y_deg");
  config.imu_euler_xyz_deg.z = parser.get("imu_euler_z_deg");

  return config;
}
