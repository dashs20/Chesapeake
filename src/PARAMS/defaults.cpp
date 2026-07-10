#include "defaults.hpp"

void load_default_config(MASTERc& config) {
    config.magic = 0x43484557;

    config.halc.motc.m1_pin = 27;
    config.halc.motc.m2_pin = 28;
    config.halc.motc.m3_pin = 5;
    config.halc.motc.m4_pin = 6;
    config.halc.motc.speed_kbd = 300;
    config.halc.motc.pole_pairs = 14;

    config.halc.rcrxc.uart_id = 0;
    config.halc.rcrxc.roll_ch = 0;
    config.halc.rcrxc.pitch_ch = 1;
    config.halc.rcrxc.thr_ch = 2;
    config.halc.rcrxc.yaw_ch = 3;
    config.halc.rcrxc.arm_ch = 4;
    config.halc.rcrxc.mode_ch = 5;

    config.halc.batc.pin = 26;
    config.halc.batc.division_factor = 10.1f;

    config.halc.imuc.spi_port = &SPI1;
    config.halc.imuc.cs_pin = 9;
    config.halc.imuc.accel_fs = LSM6DSV16X_ACC_FS::FS_8G;
    config.halc.imuc.gyro_fs = LSM6DSV16X_GYRO_FS::FS_1000DPS;
    config.halc.imuc.accel_odr = LSM6DSV16X_ODR::ODR_480Hz;
    config.halc.imuc.gyro_odr = LSM6DSV16X_ODR::ODR_480Hz;
    config.halc.imuc.accel_bias_x_mps2 = 0.0f;
    config.halc.imuc.accel_bias_y_mps2 = 0.0f;
    config.halc.imuc.accel_bias_z_mps2 = 0.0f;
    config.halc.imuc.gyro_bias_x_radps = 0.0f;
    config.halc.imuc.gyro_bias_y_radps = 0.0f;
    config.halc.imuc.gyro_bias_z_radps = 0.0f;

    config.halc.serc.s1_pin = 7;
    config.halc.serc.s2_pin = 2;
    config.halc.serc.s3_pin = 4;
    config.halc.serc.s4_pin = 3;
    config.halc.serc.min_us = 1000;
    config.halc.serc.max_us = 2000;

    config.gncc.looprate_hz = 250;
    config.gncc.allocator = ALLOCATOR::QUAD;

    Eigen::VectorXd state_x0(7);
    state_x0 << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    config.gncc.navc.x0 = state_x0;
    config.gncc.navc.r_IMU2CG_mm = Eigen::Vector3f::Zero();
    config.gncc.navc.q_IMU2body = Eigen::Quaternionf::Identity();
    config.gncc.navc.accel_bias = Eigen::Vector3f::Zero();
    config.gncc.navc.gyro_bias = Eigen::Vector3f::Zero();

    PID_SCALARc rate_pid;
    rate_pid.kp = 0.1f;
    rate_pid.ki = 0.05f;
    rate_pid.kd = 0.001f;
    rate_pid.i_max = 0.5f;
    rate_pid.out_min = -1.0f;
    rate_pid.out_max = 1.0f;
    rate_pid.dt_s = 0.0f;

    PID_SCALARc angle_pid;
    angle_pid.kp = 2.0f;
    angle_pid.ki = 0.0f;
    angle_pid.kd = 0.0f;
    angle_pid.i_max = 0.0f;
    angle_pid.out_min = -5.0f;
    angle_pid.out_max = 5.0f;
    angle_pid.dt_s = 0.0f;

    config.gncc.ctlc.angle_loop_hz = 125;
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
    config.gncc.allocc.ser_min_ang_deg = -30.0f;
    config.gncc.allocc.ser_max_ang_deg = 30.0f;
    config.gncc.allocc.ser_default_ang_deg = 0.0f;
    config.gncc.allocc.blink_hz_disarmed = 1.0f;
    config.gncc.allocc.blink_hz_rate = 5.0f;
    config.gncc.allocc.blink_hz_angle = 10.0f;

    config.halc.ledc.pin = 17;

    config.halc.rpic.enabled = true;
    config.halc.rpic.uart_id = 2;
    config.halc.rpic.baudrate = 460800;
    config.halc.rpic.tx_pin = 20;
    config.halc.rpic.rx_pin = 21;
    config.halc.rpic.rate_divisor = 5;

    config.halc.debugc.enabled = true;
    config.halc.debugc.decimation = 125;

    config.gncc.vsmc.arm_threshold_frac = 0.8f;
    config.gncc.vsmc.mode_rate_threshold_frac = 0.3f;
    config.gncc.vsmc.mode_angle_threshold_frac = 0.7f;
}
