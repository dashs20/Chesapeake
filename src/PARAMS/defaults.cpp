#include "defaults.hpp"

void load_default_config(MASTERc& config) {
    config.magic = 0x43484553;

    config.halc.motc.start_pin = 2;
    config.halc.motc.num_pins = 4;
    config.halc.motc.speed_kbd = 600;
    config.halc.motc.pole_pairs = 14;

    config.halc.rcrxc.uart_id = 1;
    config.halc.rcrxc.roll_ch = 0;
    config.halc.rcrxc.pitch_ch = 1;
    config.halc.rcrxc.thr_ch = 2;
    config.halc.rcrxc.yaw_ch = 3;
    config.halc.rcrxc.arm_ch = 4;
    config.halc.rcrxc.mode_ch = 5;

    config.halc.batc.pin = 26;
    config.halc.batc.division_factor = 11.0f;

    config.halc.imuc.spi_port = &SPI;
    config.halc.imuc.cs_pin = 17;
    config.halc.imuc.accel_fs = LSM6DSV16X_ACC_FS::FS_8G;
    config.halc.imuc.gyro_fs = LSM6DSV16X_GYRO_FS::FS_2000DPS;
    config.halc.imuc.accel_odr = LSM6DSV16X_ODR::ODR_480Hz;
    config.halc.imuc.gyro_odr = LSM6DSV16X_ODR::ODR_480Hz;

    config.halc.servoc.s1_pin = 255;
    config.halc.servoc.s2_pin = 255;
    config.halc.servoc.s3_pin = 255;
    config.halc.servoc.s4_pin = 255;
    config.halc.servoc.min_us = 1000;
    config.halc.servoc.max_us = 2000;

    config.gncc.dt_s = 0.002f;
    config.gncc.allocator = ALLOCATOR::QUAD;

    config.gncc.navc.dt_s = 0.002;
    Eigen::VectorXd state_x0(7);
    state_x0 << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    config.gncc.navc.x0 = state_x0;
    config.gncc.navc.r_IMU2CG_mm = Eigen::Vector3f::Zero();
    config.gncc.navc.q_IMU2body = Eigen::Quaternionf::Identity();

    PID_SCALARc rate_pid;
    rate_pid.kp = 0.1f;
    rate_pid.ki = 0.05f;
    rate_pid.kd = 0.001f;
    rate_pid.i_max = 0.5f;
    rate_pid.out_min = -1.0f;
    rate_pid.out_max = 1.0f;
    rate_pid.dt_s = 0.002f;

    PID_SCALARc angle_pid;
    angle_pid.kp = 2.0f;
    angle_pid.ki = 0.0f;
    angle_pid.kd = 0.0f;
    angle_pid.i_max = 0.0f;
    angle_pid.out_min = -5.0f;
    angle_pid.out_max = 5.0f;
    angle_pid.dt_s = 0.01f;

    config.gncc.ctlc.angle_loop_dt_s = 0.01f;
    config.gncc.ctlc.rate.roll = rate_pid;
    config.gncc.ctlc.rate.pitch = rate_pid;
    config.gncc.ctlc.rate.yaw = rate_pid;
    config.gncc.ctlc.angle.roll = angle_pid;
    config.gncc.ctlc.angle.pitch = angle_pid;
    config.gncc.ctlc.angle.yaw = angle_pid;

    config.gncc.guic.max_rate_radps = 3.14f;
    config.gncc.guic.max_angle_rad = 0.5f;
    config.gncc.guic.expoc.roll = 0.3f;
    config.gncc.guic.expoc.pitch = 0.3f;
    config.gncc.guic.expoc.yaw = 0.3f;

    config.gncc.allocc.min_motor_frac = 0.15f;
    config.gncc.allocc.servo_min_ang_deg = -30.0f;
    config.gncc.allocc.servo_max_ang_deg = 30.0f;
    config.gncc.allocc.servo_default_ang_deg = 0.0f;

    config.gncc.vsmc.arm_threshold_frac = 0.8f;
    config.gncc.vsmc.mode_rate_threshold_frac = 0.3f;
    config.gncc.vsmc.mode_angle_threshold_frac = 0.7f;
}
