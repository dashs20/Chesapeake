#include "gnc_config.hpp"

double looprate_hz = 1000.0;
double max_rate_degps = 100.0;

// define vehicle configuration
pid_cfg pid_x_cfg{.kp = 0.0022,
                  .ki = 0.0,
                  .kd = 0.0,
                  .i_lim = 20.0,
                  .looprate_hz = looprate_hz};
pid_cfg pid_y_cfg{.kp = 0.0022,
                  .ki = 0.0,
                  .kd = 0.0,
                  .i_lim = 20.0,
                  .looprate_hz = looprate_hz};
pid_cfg pid_z_cfg{.kp = 0.0001,
                  .ki = 0.0,
                  .kd = 0.0,
                  .i_lim = 20.0,
                  .looprate_hz = looprate_hz};

alloc_cfg veh_alloc_cfg{.t_min_frac = 0.05,
                        .max_delta_throttle = 0.10,
                        .theta_min_deg = 75.0,
                        .theta_max_deg = 105.0,
                        .gear_ratio = 0.5,
                        .servo1_offset_deg = 0,
                        .servo2_offset_deg = 0};
lpf_cfg imu_lpf_cfg{.fc_hz = 50.0, .looprate_hz = looprate_hz};
gnc_util::vec imu_euler_xyz_deg{.x = 0.0, .y = 270, .z = 0.0};

gnc_cfg spacey_config{.pid_x_cfg = pid_x_cfg,
                      .pid_y_cfg = pid_y_cfg,
                      .pid_z_cfg = pid_z_cfg,
                      .veh_alloc_cfg = veh_alloc_cfg,
                      .imu_lpf_cfg = imu_lpf_cfg,
                      .imu_euler_xyz_deg = imu_euler_xyz_deg};
