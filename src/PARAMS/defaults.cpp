#include "defaults.hpp"

void load_default_config(MASTERc& config) {
    config.magic = 0x4348455C;

    config.halc.motc.m1_pin = 27;
    config.halc.motc.m2_pin = 28;
    config.halc.motc.m3_pin = 5;
    config.halc.motc.m4_pin = 6;
    config.halc.motc.speed_kbd = 300;
    config.halc.motc.pole_pairs = 14;

    config.halc.rcrxc.uart_id = 1;
    config.halc.rcrxc.roll_ch = 1;
    config.halc.rcrxc.pitch_ch = 2;
    config.halc.rcrxc.thr_ch = 3;
    config.halc.rcrxc.yaw_ch = 4;
    config.halc.rcrxc.arm_ch = 5;
    config.halc.rcrxc.mode_ch = 6;
    config.halc.rcrxc.telemetry_hz = 10.0f;

    config.halc.batc.pin = 26;
    config.halc.batc.division_factor = 10.1f;

    config.halc.imuc.spi_port = &SPI1;
    config.halc.imuc.cs_pin = 9;
    config.halc.imuc.accel_fs = LSM6DSV16X_ACC_FS::FS_8G;
    config.halc.imuc.gyro_fs = LSM6DSV16X_GYRO_FS::FS_1000DPS;
    config.halc.imuc.accel_odr = LSM6DSV16X_ODR::ODR_480Hz;
    config.halc.imuc.gyro_odr = LSM6DSV16X_ODR::ODR_480Hz;

    config.halc.serc.s1_pin = 7;
    config.halc.serc.s2_pin = 2;
    config.halc.serc.s3_pin = 4;
    config.halc.serc.s4_pin = 3;
    config.halc.serc.min_us = 1000;
    config.halc.serc.max_us = 2000;
    config.halc.telemetry_uart_id = 0;
    config.halc.telemetry_decimation = 10;

    config.gncc.looprate_hz = 1000;
    config.gncc.allocator = ALLOCATOR::QUAD;

    config.gncc.navc.q_IMU2body = Eigen::Quaternionf(0.000000f, 0.707107f, 0.707107f, 0.000000f);
    config.gncc.navc.imu_calc.accel_bias = Eigen::Vector3f(-0.005663f, -0.001337f, 0.093921f);
    config.gncc.navc.imu_calc.gyro_bias = Eigen::Vector3f(0.004731f, -0.025025f, 0.012595f);
    config.gncc.navc.gyro_error_degps = 5.0f;

    PID_SCALARc angle_pid;
    angle_pid.kp = 5.0f;
    angle_pid.ki = 0.0f;
    angle_pid.kd = 0.0f;
    angle_pid.i_max = 0.0f;
    angle_pid.out_min = -1.0f;
    angle_pid.out_max = 1.0f;
    angle_pid.dt_s = 0.0f;

    PID_SCALARc rate_pid;
    rate_pid.kp = 15.0f;
    rate_pid.ki = 0.10f;
    rate_pid.kd = 0.0f;
    rate_pid.i_max = 2.0f;
    rate_pid.out_min = -1.0f;
    rate_pid.out_max = 1.0f;
    rate_pid.dt_s = 0.0f;

    PID_SCALARc yaw_rate_pid;
    yaw_rate_pid.kp = 12.0f;
    yaw_rate_pid.ki = 0.10f;
    yaw_rate_pid.kd = 0.0f;
    yaw_rate_pid.i_max = 10.0f;
    yaw_rate_pid.out_min = -1.0f;
    yaw_rate_pid.out_max = 1.0f;
    yaw_rate_pid.dt_s = 0.0f;

    config.gncc.ctlc.angle_loop_hz = 500;
    config.gncc.ctlc.rate.roll = rate_pid;
    config.gncc.ctlc.rate.pitch = rate_pid;

    config.gncc.ctlc.rate.yaw = yaw_rate_pid;
    config.gncc.ctlc.angle.roll = angle_pid;
    config.gncc.ctlc.angle.pitch = angle_pid;
    config.gncc.ctlc.angle.yaw = angle_pid;

    config.gncc.guic.max_rate_radps = 3.14f;
    config.gncc.guic.max_angle_rad = 0.5f;
    config.gncc.guic.expoc.roll = 0.3f;
    config.gncc.guic.expoc.pitch = 0.3f;
    config.gncc.guic.expoc.yaw = 0.3f;

    config.gncc.allocc.min_motor_frac = 0.05f;
    config.gncc.allocc.max_motor_frac = 0.66f;
    config.gncc.allocc.ser_min_ang_deg = -30.0f;
    config.gncc.allocc.ser_max_ang_deg = 30.0f;
    config.gncc.allocc.ser_default_ang_deg = 90.0f;
    config.gncc.allocc.blink_hz_disarmed = 1.0f;
    config.gncc.allocc.blink_hz_rate = 5.0f;
    config.gncc.allocc.blink_hz_angle = 10.0f;

    config.halc.ledc.pin = 17;

    config.gncc.vsmc.arm_threshold_frac = 0.8f;
    config.gncc.vsmc.mode_rate_threshold_frac = 0.3f;
    config.gncc.vsmc.mode_angle_threshold_frac = 0.7f;
}
