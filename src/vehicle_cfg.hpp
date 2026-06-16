#pragma once
#include "gnc/gnc.hpp"
#include "gnc/gnc_util/gnc_util.hpp"

// config file for vehicle

double looprate_hz = 1000.0;

// define vehicle configuration
// PID
pid_cfg pid_x_cfg{
    .kp = 1.0, .ki = 0.0, .kd = 0.0, .i_lim = 20.0, .looprate_hz = looprate_hz};
pid_cfg pid_y_cfg{
    .kp = 1.0, .ki = 0.0, .kd = 0.0, .i_lim = 20.0, .looprate_hz = looprate_hz};
pid_cfg pid_z_cfg{
    .kp = 1.0, .ki = 0.0, .kd = 0.0, .i_lim = 20.0, .looprate_hz = looprate_hz};

// Allocator
alloc_cfg veh_alloc_cfg{.t_min_frac = 0.05,
                        .theta_min_deg = 20.0,
                        .theta_max_deg = 70.0,
                        .gear_ratio = 16.0 / 30.0};

// Filter
lpf_cfg imu_lpf_cfg{.fc_hz = 50.0, .looprate_hz = looprate_hz};

// IMU orientation
gnc_util::vec imu_euler_xyz_deg{.x = 180.0, .y = 0.0, .z = -45.0};

// main config struct
gnc_cfg cfg{.pid_x_cfg = pid_x_cfg,
            .pid_y_cfg = pid_y_cfg,
            .pid_z_cfg = pid_z_cfg,
            .veh_alloc_cfg = veh_alloc_cfg,
            .imu_lpf_cfg = imu_lpf_cfg,
            .imu_euler_xyz_deg = imu_euler_xyz_deg};
