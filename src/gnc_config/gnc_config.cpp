#include "gnc_config.hpp"

double looprate_hz = 1000.0;
double max_rate_degps = 100.0;

// define vehicle configuration
pid_cfg pid_x_cfg{.kp = 0.0005,
                  .ki = 0.0,
                  .kd = 0.0,
                  .i_lim = 20.0,
                  .looprate_hz = looprate_hz};
pid_cfg pid_y_cfg{
    .kp = 0.0, .ki = 0.0, .kd = 0.0, .i_lim = 20.0, .looprate_hz = looprate_hz};
pid_cfg pid_z_cfg{
    .kp = 0.0, .ki = 0.0, .kd = 0.0, .i_lim = 20.0, .looprate_hz = looprate_hz};

alloc_cfg veh_alloc_cfg{.t_min_frac = 0.05,
                        .theta_min_deg = 20.0,
                        .theta_max_deg = 70.0,
                        .gear_ratio = 16.0 / 30.0,
                        .servo1_offset_deg = 2,
                        .servo2_offset_deg = -4};
lpf_cfg imu_lpf_cfg{.fc_hz = 50.0, .looprate_hz = looprate_hz};
gnc_util::vec imu_euler_xyz_deg{.x = 180.0, .y = 0.0, .z = 135};

gnc_cfg spacey_config{.pid_x_cfg = pid_x_cfg,
                      .pid_y_cfg = pid_y_cfg,
                      .pid_z_cfg = pid_z_cfg,
                      .veh_alloc_cfg = veh_alloc_cfg,
                      .imu_lpf_cfg = imu_lpf_cfg,
                      .imu_euler_xyz_deg = imu_euler_xyz_deg};
