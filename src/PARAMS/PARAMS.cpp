#include "PARAMS.hpp"
#include "defaults.hpp"
#include <Arduino.h>
#include <cstring>
#include <cstdlib>
#include <algorithm>
#include "../HAL/HAL.hpp"
#include "hardware/watchdog.h"
#include <pico/multicore.h>

extern HAL* hal_ptr;

PARAMS::PARAMS() : is_calibrating(false), calibration_counter(0), sum_ax(0.0f), sum_ay(0.0f), sum_az(0.0f), sum_gx(0.0f), sum_gy(0.0f), sum_gz(0.0f) {
    EEPROM.begin(sizeof(MASTERc));
}

PARAMS::~PARAMS() {}

bool PARAMS::load(MASTERc& config) {
    EEPROM.get(0, config);
    config.halc.imuc.spi_port = &SPI1;
    Serial.printf("DEBUG: Loaded config. Magic = 0x%08X, Size = %u\n", config.magic, (unsigned int)sizeof(MASTERc));
    return (config.magic == 0x43484556);
}

extern volatile bool system_ready;

void PARAMS::save(const MASTERc& config) {
    system_ready = false;
    delay(50);
    MASTERc mutable_config = config;
    mutable_config.magic = 0x43484556;
    EEPROM.put(0, mutable_config);
    bool success = EEPROM.commit();
    Serial.printf("DEBUG: EEPROM.commit() success = %d\n", success);
    system_ready = true;
}

void PARAMS::print_help() {
    Serial.println("Chesapeake CLI Help (For Chesapeake supported boards, like Bluecrab)");
    Serial.println("Commands:");
    Serial.println("  help                       - Show this help list");
    Serial.println("  dump                       - Dump all parameter settings and current values");
    Serial.println("  get <param>                - Print the current value of a parameter");
    Serial.println("  set <param> = <value>      - Set a parameter in RAM (no spaces around '=')");
    Serial.println("  save                       - Save RAM configuration to flash and reboot");
    Serial.println("  defaults                   - Load default settings into RAM (requires save)");
    Serial.println("  calibrate                  - Calibrate IMU/UKF flat and save biases");
    Serial.println("  reboot                     - Reboot the flight controller");
}

void PARAMS::print_all(const MASTERc& config) {
    Serial.print("mot_m1_pin = "); Serial.println(config.halc.motc.m1_pin);
    Serial.print("mot_m2_pin = "); Serial.println(config.halc.motc.m2_pin);
    Serial.print("mot_m3_pin = "); Serial.println(config.halc.motc.m3_pin);
    Serial.print("mot_m4_pin = "); Serial.println(config.halc.motc.m4_pin);
    Serial.print("mot_speed_kbd = "); Serial.println(config.halc.motc.speed_kbd);
    Serial.print("mot_pole_pairs = "); Serial.println(config.halc.motc.pole_pairs);

    Serial.print("rcrx_uart_id = "); Serial.println(config.halc.rcrxc.uart_id);
    Serial.print("rcrx_roll_ch = "); Serial.println(config.halc.rcrxc.roll_ch);
    Serial.print("rcrx_pitch_ch = "); Serial.println(config.halc.rcrxc.pitch_ch);
    Serial.print("rcrx_thr_ch = "); Serial.println(config.halc.rcrxc.thr_ch);
    Serial.print("rcrx_yaw_ch = "); Serial.println(config.halc.rcrxc.yaw_ch);
    Serial.print("rcrx_arm_ch = "); Serial.println(config.halc.rcrxc.arm_ch);
    Serial.print("rcrx_mode_ch = "); Serial.println(config.halc.rcrxc.mode_ch);

    Serial.print("bat_pin = "); Serial.println(config.halc.batc.pin);
    Serial.print("bat_division_factor = "); Serial.println(config.halc.batc.division_factor);

    Serial.print("imu_cs_pin = "); Serial.println(config.halc.imuc.cs_pin);
    Serial.print("imu_accel_fs = ");
    switch (config.halc.imuc.accel_fs) {
        case LSM6DSV16X_ACC_FS::FS_2G: Serial.println("2G"); break;
        case LSM6DSV16X_ACC_FS::FS_4G: Serial.println("4G"); break;
        case LSM6DSV16X_ACC_FS::FS_8G: Serial.println("8G"); break;
        case LSM6DSV16X_ACC_FS::FS_16G: Serial.println("16G"); break;
    }
    Serial.print("imu_gyro_fs = ");
    switch (config.halc.imuc.gyro_fs) {
        case LSM6DSV16X_GYRO_FS::FS_125DPS: Serial.println("125DPS"); break;
        case LSM6DSV16X_GYRO_FS::FS_250DPS: Serial.println("250DPS"); break;
        case LSM6DSV16X_GYRO_FS::FS_500DPS: Serial.println("500DPS"); break;
        case LSM6DSV16X_GYRO_FS::FS_1000DPS: Serial.println("1000DPS"); break;
        case LSM6DSV16X_GYRO_FS::FS_2000DPS: Serial.println("2000DPS"); break;
        case LSM6DSV16X_GYRO_FS::FS_4000DPS: Serial.println("4000DPS"); break;
    }
    Serial.print("imu_accel_odr = ");
    switch (config.halc.imuc.accel_odr) {
        case LSM6DSV16X_ODR::OFF: Serial.println("OFF"); break;
        case LSM6DSV16X_ODR::ODR_1Hz875: Serial.println("1.875Hz"); break;
        case LSM6DSV16X_ODR::ODR_7Hz5: Serial.println("7.5Hz"); break;
        case LSM6DSV16X_ODR::ODR_15Hz: Serial.println("15Hz"); break;
        case LSM6DSV16X_ODR::ODR_30Hz: Serial.println("30Hz"); break;
        case LSM6DSV16X_ODR::ODR_60Hz: Serial.println("60Hz"); break;
        case LSM6DSV16X_ODR::ODR_120Hz: Serial.println("120Hz"); break;
        case LSM6DSV16X_ODR::ODR_240Hz: Serial.println("240Hz"); break;
        case LSM6DSV16X_ODR::ODR_480Hz: Serial.println("480Hz"); break;
        case LSM6DSV16X_ODR::ODR_960Hz: Serial.println("960Hz"); break;
        case LSM6DSV16X_ODR::ODR_1920Hz: Serial.println("1920Hz"); break;
        case LSM6DSV16X_ODR::ODR_3840Hz: Serial.println("3840Hz"); break;
        case LSM6DSV16X_ODR::ODR_7680Hz: Serial.println("7680Hz"); break;
    }

    Serial.print("imu_gyro_odr = ");
    switch (config.halc.imuc.gyro_odr) {
        case LSM6DSV16X_ODR::OFF: Serial.println("OFF"); break;
        case LSM6DSV16X_ODR::ODR_1Hz875: Serial.println("1.875Hz"); break;
        case LSM6DSV16X_ODR::ODR_7Hz5: Serial.println("7.5Hz"); break;
        case LSM6DSV16X_ODR::ODR_15Hz: Serial.println("15Hz"); break;
        case LSM6DSV16X_ODR::ODR_30Hz: Serial.println("30Hz"); break;
        case LSM6DSV16X_ODR::ODR_60Hz: Serial.println("60Hz"); break;
        case LSM6DSV16X_ODR::ODR_120Hz: Serial.println("120Hz"); break;
        case LSM6DSV16X_ODR::ODR_240Hz: Serial.println("240Hz"); break;
        case LSM6DSV16X_ODR::ODR_480Hz: Serial.println("480Hz"); break;
        case LSM6DSV16X_ODR::ODR_960Hz: Serial.println("960Hz"); break;
        case LSM6DSV16X_ODR::ODR_1920Hz: Serial.println("1920Hz"); break;
        case LSM6DSV16X_ODR::ODR_3840Hz: Serial.println("3840Hz"); break;
        case LSM6DSV16X_ODR::ODR_7680Hz: Serial.println("7680Hz"); break;
    }
    Serial.print("imu_accel_bias_x_mps2 = "); Serial.println(config.halc.imuc.accel_bias_x_mps2, 6);
    Serial.print("imu_accel_bias_y_mps2 = "); Serial.println(config.halc.imuc.accel_bias_y_mps2, 6);
    Serial.print("imu_accel_bias_z_mps2 = "); Serial.println(config.halc.imuc.accel_bias_z_mps2, 6);
    Serial.print("imu_gyro_bias_x_radps = "); Serial.println(config.halc.imuc.gyro_bias_x_radps, 6);
    Serial.print("imu_gyro_bias_y_radps = "); Serial.println(config.halc.imuc.gyro_bias_y_radps, 6);
    Serial.print("imu_gyro_bias_z_radps = "); Serial.println(config.halc.imuc.gyro_bias_z_radps, 6);


    Serial.print("ser_s1_pin = "); Serial.println(config.halc.serc.s1_pin);
    Serial.print("ser_s2_pin = "); Serial.println(config.halc.serc.s2_pin);
    Serial.print("ser_s3_pin = "); Serial.println(config.halc.serc.s3_pin);
    Serial.print("ser_s4_pin = "); Serial.println(config.halc.serc.s4_pin);
    Serial.print("ser_min_us = "); Serial.println(config.halc.serc.min_us);
    Serial.print("ser_max_us = "); Serial.println(config.halc.serc.max_us);
    Serial.print("led_pin = "); Serial.println(config.halc.ledc.pin);
    Serial.print("rpi_enabled = "); Serial.println(config.halc.rpic.enabled);
    Serial.print("rpi_uart_id = "); Serial.println(config.halc.rpic.uart_id);
    Serial.print("rpi_baudrate = "); Serial.println(config.halc.rpic.baudrate);
    Serial.print("rpi_tx_pin = "); Serial.println(config.halc.rpic.tx_pin);
    Serial.print("rpi_rx_pin = "); Serial.println(config.halc.rpic.rx_pin);
    Serial.print("rpi_rate_divisor = "); Serial.println(config.halc.rpic.rate_divisor);
    Serial.print("debug_enabled = "); Serial.println(config.halc.debugc.enabled);
    Serial.print("debug_decimation = "); Serial.println(config.halc.debugc.decimation);

    Serial.print("gnc_looprate_hz = "); Serial.println(config.gncc.looprate_hz);
    Serial.print("angle_loop_hz = "); Serial.println(config.gncc.ctlc.angle_loop_hz);
    Serial.print("roll_rate_kp = "); Serial.println(config.gncc.ctlc.rate.roll.kp);
    Serial.print("roll_rate_ki = "); Serial.println(config.gncc.ctlc.rate.roll.ki);
    Serial.print("roll_rate_kd = "); Serial.println(config.gncc.ctlc.rate.roll.kd);
    Serial.print("roll_rate_imax = "); Serial.println(config.gncc.ctlc.rate.roll.i_max);
    Serial.print("pitch_rate_kp = "); Serial.println(config.gncc.ctlc.rate.pitch.kp);
    Serial.print("pitch_rate_ki = "); Serial.println(config.gncc.ctlc.rate.pitch.ki);
    Serial.print("pitch_rate_kd = "); Serial.println(config.gncc.ctlc.rate.pitch.kd);
    Serial.print("pitch_rate_imax = "); Serial.println(config.gncc.ctlc.rate.pitch.i_max);
    Serial.print("yaw_rate_kp = "); Serial.println(config.gncc.ctlc.rate.yaw.kp);
    Serial.print("yaw_rate_ki = "); Serial.println(config.gncc.ctlc.rate.yaw.ki);
    Serial.print("yaw_rate_kd = "); Serial.println(config.gncc.ctlc.rate.yaw.kd);
    Serial.print("yaw_rate_imax = "); Serial.println(config.gncc.ctlc.rate.yaw.i_max);

    Serial.print("roll_ang_kp = "); Serial.println(config.gncc.ctlc.angle.roll.kp);
    Serial.print("pitch_ang_kp = "); Serial.println(config.gncc.ctlc.angle.pitch.kp);
    Serial.print("yaw_ang_kp = "); Serial.println(config.gncc.ctlc.angle.yaw.kp);
    Serial.print("blink_hz_disarmed = "); Serial.println(config.gncc.allocc.blink_hz_disarmed);
    Serial.print("blink_hz_rate = "); Serial.println(config.gncc.allocc.blink_hz_rate);
    Serial.print("blink_hz_angle = "); Serial.println(config.gncc.allocc.blink_hz_angle);
    Serial.print("gnc_nav_accel_bias_x = "); Serial.println(config.gncc.navc.accel_bias.x(), 6);
    Serial.print("gnc_nav_accel_bias_y = "); Serial.println(config.gncc.navc.accel_bias.y(), 6);
    Serial.print("gnc_nav_accel_bias_z = "); Serial.println(config.gncc.navc.accel_bias.z(), 6);
    Serial.print("gnc_nav_gyro_bias_x = "); Serial.println(config.gncc.navc.gyro_bias.x(), 6);
    Serial.print("gnc_nav_gyro_bias_y = "); Serial.println(config.gncc.navc.gyro_bias.y(), 6);
    Serial.print("gnc_nav_gyro_bias_z = "); Serial.println(config.gncc.navc.gyro_bias.z(), 6);
    Serial.println("--- END OF DUMP ---");
}

void PARAMS::get_parameter(const MASTERc& config, const char* name) {
    if (std::strcmp(name, "mot_m1_pin") == 0) {
        Serial.println(config.halc.motc.m1_pin);
    } else if (std::strcmp(name, "mot_m2_pin") == 0) {
        Serial.println(config.halc.motc.m2_pin);
    } else if (std::strcmp(name, "mot_m3_pin") == 0) {
        Serial.println(config.halc.motc.m3_pin);
    } else if (std::strcmp(name, "mot_m4_pin") == 0) {
        Serial.println(config.halc.motc.m4_pin);
    } else if (std::strcmp(name, "mot_speed_kbd") == 0) {
        Serial.println(config.halc.motc.speed_kbd);
    } else if (std::strcmp(name, "mot_pole_pairs") == 0) {
        Serial.println(config.halc.motc.pole_pairs);
    } else if (std::strcmp(name, "rcrx_uart_id") == 0) {
        Serial.println(config.halc.rcrxc.uart_id);
    } else if (std::strcmp(name, "rcrx_roll_ch") == 0) {
        Serial.println(config.halc.rcrxc.roll_ch);
    } else if (std::strcmp(name, "rcrx_pitch_ch") == 0) {
        Serial.println(config.halc.rcrxc.pitch_ch);
    } else if (std::strcmp(name, "rcrx_thr_ch") == 0) {
        Serial.println(config.halc.rcrxc.thr_ch);
    } else if (std::strcmp(name, "rcrx_yaw_ch") == 0) {
        Serial.println(config.halc.rcrxc.yaw_ch);
    } else if (std::strcmp(name, "rcrx_arm_ch") == 0) {
        Serial.println(config.halc.rcrxc.arm_ch);
    } else if (std::strcmp(name, "rcrx_mode_ch") == 0) {
        Serial.println(config.halc.rcrxc.mode_ch);
    } else if (std::strcmp(name, "bat_pin") == 0) {
        Serial.println(config.halc.batc.pin);
    } else if (std::strcmp(name, "bat_division_factor") == 0) {
        Serial.println(config.halc.batc.division_factor);
    } else if (std::strcmp(name, "imu_cs_pin") == 0) {
        Serial.println(config.halc.imuc.cs_pin);
    } else if (std::strcmp(name, "imu_accel_fs") == 0) {
        switch (config.halc.imuc.accel_fs) {
            case LSM6DSV16X_ACC_FS::FS_2G: Serial.println("2G"); break;
            case LSM6DSV16X_ACC_FS::FS_4G: Serial.println("4G"); break;
            case LSM6DSV16X_ACC_FS::FS_8G: Serial.println("8G"); break;
            case LSM6DSV16X_ACC_FS::FS_16G: Serial.println("16G"); break;
        }
    } else if (std::strcmp(name, "imu_gyro_fs") == 0) {
        switch (config.halc.imuc.gyro_fs) {
            case LSM6DSV16X_GYRO_FS::FS_125DPS: Serial.println("125DPS"); break;
            case LSM6DSV16X_GYRO_FS::FS_250DPS: Serial.println("250DPS"); break;
            case LSM6DSV16X_GYRO_FS::FS_500DPS: Serial.println("500DPS"); break;
            case LSM6DSV16X_GYRO_FS::FS_1000DPS: Serial.println("1000DPS"); break;
            case LSM6DSV16X_GYRO_FS::FS_2000DPS: Serial.println("2000DPS"); break;
            case LSM6DSV16X_GYRO_FS::FS_4000DPS: Serial.println("4000DPS"); break;
        }
    } else if (std::strcmp(name, "imu_accel_odr") == 0) {
        switch (config.halc.imuc.accel_odr) {
            case LSM6DSV16X_ODR::OFF: Serial.println("OFF"); break;
            case LSM6DSV16X_ODR::ODR_1Hz875: Serial.println("1.875Hz"); break;
            case LSM6DSV16X_ODR::ODR_7Hz5: Serial.println("7.5Hz"); break;
            case LSM6DSV16X_ODR::ODR_15Hz: Serial.println("15Hz"); break;
            case LSM6DSV16X_ODR::ODR_30Hz: Serial.println("30Hz"); break;
            case LSM6DSV16X_ODR::ODR_60Hz: Serial.println("60Hz"); break;
            case LSM6DSV16X_ODR::ODR_120Hz: Serial.println("120Hz"); break;
            case LSM6DSV16X_ODR::ODR_240Hz: Serial.println("240Hz"); break;
            case LSM6DSV16X_ODR::ODR_480Hz: Serial.println("480Hz"); break;
            case LSM6DSV16X_ODR::ODR_960Hz: Serial.println("960Hz"); break;
            case LSM6DSV16X_ODR::ODR_1920Hz: Serial.println("1920Hz"); break;
            case LSM6DSV16X_ODR::ODR_3840Hz: Serial.println("3840Hz"); break;
            case LSM6DSV16X_ODR::ODR_7680Hz: Serial.println("7680Hz"); break;
        }
    } else if (std::strcmp(name, "imu_gyro_odr") == 0) {
        switch (config.halc.imuc.gyro_odr) {
            case LSM6DSV16X_ODR::OFF: Serial.println("OFF"); break;
            case LSM6DSV16X_ODR::ODR_1Hz875: Serial.println("1.875Hz"); break;
            case LSM6DSV16X_ODR::ODR_7Hz5: Serial.println("7.5Hz"); break;
            case LSM6DSV16X_ODR::ODR_15Hz: Serial.println("15Hz"); break;
            case LSM6DSV16X_ODR::ODR_30Hz: Serial.println("30Hz"); break;
            case LSM6DSV16X_ODR::ODR_60Hz: Serial.println("60Hz"); break;
            case LSM6DSV16X_ODR::ODR_120Hz: Serial.println("120Hz"); break;
            case LSM6DSV16X_ODR::ODR_240Hz: Serial.println("240Hz"); break;
            case LSM6DSV16X_ODR::ODR_480Hz: Serial.println("480Hz"); break;
            case LSM6DSV16X_ODR::ODR_960Hz: Serial.println("960Hz"); break;
            case LSM6DSV16X_ODR::ODR_1920Hz: Serial.println("1920Hz"); break;
            case LSM6DSV16X_ODR::ODR_3840Hz: Serial.println("3840Hz"); break;
            case LSM6DSV16X_ODR::ODR_7680Hz: Serial.println("7680Hz"); break;
        }
    } else if (std::strcmp(name, "imu_accel_bias_x_mps2") == 0) {
        Serial.println(config.halc.imuc.accel_bias_x_mps2, 6);
    } else if (std::strcmp(name, "imu_accel_bias_y_mps2") == 0) {
        Serial.println(config.halc.imuc.accel_bias_y_mps2, 6);
    } else if (std::strcmp(name, "imu_accel_bias_z_mps2") == 0) {
        Serial.println(config.halc.imuc.accel_bias_z_mps2, 6);
    } else if (std::strcmp(name, "imu_gyro_bias_x_radps") == 0) {
        Serial.println(config.halc.imuc.gyro_bias_x_radps, 6);
    } else if (std::strcmp(name, "imu_gyro_bias_y_radps") == 0) {
        Serial.println(config.halc.imuc.gyro_bias_y_radps, 6);
    } else if (std::strcmp(name, "imu_gyro_bias_z_radps") == 0) {
        Serial.println(config.halc.imuc.gyro_bias_z_radps, 6);
    } else if (std::strcmp(name, "ser_s1_pin") == 0) {
        Serial.println(config.halc.serc.s1_pin);
    } else if (std::strcmp(name, "ser_s2_pin") == 0) {
        Serial.println(config.halc.serc.s2_pin);
    } else if (std::strcmp(name, "ser_s3_pin") == 0) {
        Serial.println(config.halc.serc.s3_pin);
    } else if (std::strcmp(name, "ser_s4_pin") == 0) {
        Serial.println(config.halc.serc.s4_pin);
    } else if (std::strcmp(name, "ser_min_us") == 0) {
        Serial.println(config.halc.serc.min_us);
    } else if (std::strcmp(name, "ser_max_us") == 0) {
        Serial.println(config.halc.serc.max_us);
    } else if (std::strcmp(name, "led_pin") == 0) {
        Serial.println(config.halc.ledc.pin);
    } else if (std::strcmp(name, "rpi_enabled") == 0) {
        Serial.println(config.halc.rpic.enabled);
    } else if (std::strcmp(name, "rpi_uart_id") == 0) {
        Serial.println(config.halc.rpic.uart_id);
    } else if (std::strcmp(name, "rpi_baudrate") == 0) {
        Serial.println(config.halc.rpic.baudrate);
    } else if (std::strcmp(name, "rpi_tx_pin") == 0) {
        Serial.println(config.halc.rpic.tx_pin);
    } else if (std::strcmp(name, "rpi_rx_pin") == 0) {
        Serial.println(config.halc.rpic.rx_pin);
    } else if (std::strcmp(name, "rpi_rate_divisor") == 0) {
        Serial.println(config.halc.rpic.rate_divisor);
    } else if (std::strcmp(name, "debug_enabled") == 0) {
        Serial.println(config.halc.debugc.enabled);
    } else if (std::strcmp(name, "debug_decimation") == 0) {
        Serial.println(config.halc.debugc.decimation);
    } else if (std::strcmp(name, "blink_hz_disarmed") == 0) {
        Serial.println(config.gncc.allocc.blink_hz_disarmed);
    } else if (std::strcmp(name, "blink_hz_rate") == 0) {
        Serial.println(config.gncc.allocc.blink_hz_rate);
    } else if (std::strcmp(name, "blink_hz_angle") == 0) {
        Serial.println(config.gncc.allocc.blink_hz_angle);
    } else if (std::strcmp(name, "gnc_looprate_hz") == 0) {
        Serial.println(config.gncc.looprate_hz);
    } else if (std::strcmp(name, "angle_loop_hz") == 0) {
        Serial.println(config.gncc.ctlc.angle_loop_hz);
    } else if (std::strcmp(name, "roll_rate_kp") == 0) {
        Serial.println(config.gncc.ctlc.rate.roll.kp);
    } else if (std::strcmp(name, "roll_rate_ki") == 0) {
        Serial.println(config.gncc.ctlc.rate.roll.ki);
    } else if (std::strcmp(name, "roll_rate_kd") == 0) {
        Serial.println(config.gncc.ctlc.rate.roll.kd);
    } else if (std::strcmp(name, "roll_rate_imax") == 0) {
        Serial.println(config.gncc.ctlc.rate.roll.i_max);
    } else if (std::strcmp(name, "pitch_rate_kp") == 0) {
        Serial.println(config.gncc.ctlc.rate.pitch.kp);
    } else if (std::strcmp(name, "pitch_rate_ki") == 0) {
        Serial.println(config.gncc.ctlc.rate.pitch.ki);
    } else if (std::strcmp(name, "pitch_rate_kd") == 0) {
        Serial.println(config.gncc.ctlc.rate.pitch.kd);
    } else if (std::strcmp(name, "pitch_rate_imax") == 0) {
        Serial.println(config.gncc.ctlc.rate.pitch.i_max);
    } else if (std::strcmp(name, "yaw_rate_kp") == 0) {
        Serial.println(config.gncc.ctlc.rate.yaw.kp);
    } else if (std::strcmp(name, "yaw_rate_ki") == 0) {
        Serial.println(config.gncc.ctlc.rate.yaw.ki);
    } else if (std::strcmp(name, "yaw_rate_kd") == 0) {
        Serial.println(config.gncc.ctlc.rate.yaw.kd);
    } else if (std::strcmp(name, "yaw_rate_imax") == 0) {
        Serial.println(config.gncc.ctlc.rate.yaw.i_max);
    } else if (std::strcmp(name, "roll_ang_kp") == 0) {
        Serial.println(config.gncc.ctlc.angle.roll.kp);
    } else if (std::strcmp(name, "pitch_ang_kp") == 0) {
        Serial.println(config.gncc.ctlc.angle.pitch.kp);
    } else if (std::strcmp(name, "yaw_ang_kp") == 0) {
        Serial.println(config.gncc.ctlc.angle.yaw.kp);
    } else if (std::strcmp(name, "gnc_nav_accel_bias_x") == 0) {
        Serial.println(config.gncc.navc.accel_bias.x(), 6);
    } else if (std::strcmp(name, "gnc_nav_accel_bias_y") == 0) {
        Serial.println(config.gncc.navc.accel_bias.y(), 6);
    } else if (std::strcmp(name, "gnc_nav_accel_bias_z") == 0) {
        Serial.println(config.gncc.navc.accel_bias.z(), 6);
    } else if (std::strcmp(name, "gnc_nav_gyro_bias_x") == 0) {
        Serial.println(config.gncc.navc.gyro_bias.x(), 6);
    } else if (std::strcmp(name, "gnc_nav_gyro_bias_y") == 0) {
        Serial.println(config.gncc.navc.gyro_bias.y(), 6);
    } else if (std::strcmp(name, "gnc_nav_gyro_bias_z") == 0) {
        Serial.println(config.gncc.navc.gyro_bias.z(), 6);
    } else {
        Serial.println("Error: Unknown parameter");
    }
}

void PARAMS::set_parameter(MASTERc& config, const char* name, const char* value) {
    if (std::strcmp(name, "mot_m1_pin") == 0) {
        config.halc.motc.m1_pin = static_cast<uint8_t>(std::strtol(value, nullptr, 10));
    } else if (std::strcmp(name, "mot_m2_pin") == 0) {
        config.halc.motc.m2_pin = static_cast<uint8_t>(std::strtol(value, nullptr, 10));
    } else if (std::strcmp(name, "mot_m3_pin") == 0) {
        config.halc.motc.m3_pin = static_cast<uint8_t>(std::strtol(value, nullptr, 10));
    } else if (std::strcmp(name, "mot_m4_pin") == 0) {
        config.halc.motc.m4_pin = static_cast<uint8_t>(std::strtol(value, nullptr, 10));
    } else if (std::strcmp(name, "mot_speed_kbd") == 0) {
        config.halc.motc.speed_kbd = static_cast<uint32_t>(std::strtol(value, nullptr, 10));
    } else if (std::strcmp(name, "mot_pole_pairs") == 0) {
        config.halc.motc.pole_pairs = static_cast<uint8_t>(std::strtol(value, nullptr, 10));
    } else if (std::strcmp(name, "rcrx_uart_id") == 0) {
        config.halc.rcrxc.uart_id = static_cast<uint8_t>(std::strtol(value, nullptr, 10));
    } else if (std::strcmp(name, "rcrx_roll_ch") == 0) {
        config.halc.rcrxc.roll_ch = static_cast<uint8_t>(std::strtol(value, nullptr, 10));
    } else if (std::strcmp(name, "rcrx_pitch_ch") == 0) {
        config.halc.rcrxc.pitch_ch = static_cast<uint8_t>(std::strtol(value, nullptr, 10));
    } else if (std::strcmp(name, "rcrx_thr_ch") == 0) {
        config.halc.rcrxc.thr_ch = static_cast<uint8_t>(std::strtol(value, nullptr, 10));
    } else if (std::strcmp(name, "rcrx_yaw_ch") == 0) {
        config.halc.rcrxc.yaw_ch = static_cast<uint8_t>(std::strtol(value, nullptr, 10));
    } else if (std::strcmp(name, "rcrx_arm_ch") == 0) {
        config.halc.rcrxc.arm_ch = static_cast<uint8_t>(std::strtol(value, nullptr, 10));
    } else if (std::strcmp(name, "rcrx_mode_ch") == 0) {
        config.halc.rcrxc.mode_ch = static_cast<uint8_t>(std::strtol(value, nullptr, 10));
    } else if (std::strcmp(name, "bat_pin") == 0) {
        config.halc.batc.pin = static_cast<uint8_t>(std::strtol(value, nullptr, 10));
    } else if (std::strcmp(name, "bat_division_factor") == 0) {
        config.halc.batc.division_factor = static_cast<float>(std::strtod(value, nullptr));
    } else if (std::strcmp(name, "imu_cs_pin") == 0) {
        config.halc.imuc.cs_pin = static_cast<uint8_t>(std::strtol(value, nullptr, 10));
    } else if (std::strcmp(name, "imu_accel_fs") == 0) {
        if (std::strcmp(value, "2G") == 0 || std::strcmp(value, "2") == 0) {
            config.halc.imuc.accel_fs = LSM6DSV16X_ACC_FS::FS_2G;
        } else if (std::strcmp(value, "4G") == 0 || std::strcmp(value, "4") == 0) {
            config.halc.imuc.accel_fs = LSM6DSV16X_ACC_FS::FS_4G;
        } else if (std::strcmp(value, "8G") == 0 || std::strcmp(value, "8") == 0) {
            config.halc.imuc.accel_fs = LSM6DSV16X_ACC_FS::FS_8G;
        } else if (std::strcmp(value, "16G") == 0 || std::strcmp(value, "16") == 0) {
            config.halc.imuc.accel_fs = LSM6DSV16X_ACC_FS::FS_16G;
        } else {
            Serial.println("Error: Invalid accel range");
            return;
        }
    } else if (std::strcmp(name, "imu_gyro_fs") == 0) {
        if (std::strcmp(value, "125") == 0 || std::strcmp(value, "125DPS") == 0) {
            config.halc.imuc.gyro_fs = LSM6DSV16X_GYRO_FS::FS_125DPS;
        } else if (std::strcmp(value, "250") == 0 || std::strcmp(value, "250DPS") == 0) {
            config.halc.imuc.gyro_fs = LSM6DSV16X_GYRO_FS::FS_250DPS;
        } else if (std::strcmp(value, "500") == 0 || std::strcmp(value, "500DPS") == 0) {
            config.halc.imuc.gyro_fs = LSM6DSV16X_GYRO_FS::FS_500DPS;
        } else if (std::strcmp(value, "1000") == 0 || std::strcmp(value, "1000DPS") == 0) {
            config.halc.imuc.gyro_fs = LSM6DSV16X_GYRO_FS::FS_1000DPS;
        } else if (std::strcmp(value, "2000") == 0 || std::strcmp(value, "2000DPS") == 0) {
            config.halc.imuc.gyro_fs = LSM6DSV16X_GYRO_FS::FS_2000DPS;
        } else if (std::strcmp(value, "4000") == 0 || std::strcmp(value, "4000DPS") == 0) {
            config.halc.imuc.gyro_fs = LSM6DSV16X_GYRO_FS::FS_4000DPS;
        } else {
            Serial.println("Error: Invalid gyro range");
            return;
        }
    } else if (std::strcmp(name, "imu_accel_odr") == 0) {
        if (std::strcmp(value, "OFF") == 0) config.halc.imuc.accel_odr = LSM6DSV16X_ODR::OFF;
        else if (std::strcmp(value, "1.875Hz") == 0 || std::strcmp(value, "1.875") == 0) config.halc.imuc.accel_odr = LSM6DSV16X_ODR::ODR_1Hz875;
        else if (std::strcmp(value, "7.5Hz") == 0 || std::strcmp(value, "7.5") == 0) config.halc.imuc.accel_odr = LSM6DSV16X_ODR::ODR_7Hz5;
        else if (std::strcmp(value, "15Hz") == 0 || std::strcmp(value, "15") == 0) config.halc.imuc.accel_odr = LSM6DSV16X_ODR::ODR_15Hz;
        else if (std::strcmp(value, "30Hz") == 0 || std::strcmp(value, "30") == 0) config.halc.imuc.accel_odr = LSM6DSV16X_ODR::ODR_30Hz;
        else if (std::strcmp(value, "60Hz") == 0 || std::strcmp(value, "60") == 0) config.halc.imuc.accel_odr = LSM6DSV16X_ODR::ODR_60Hz;
        else if (std::strcmp(value, "120Hz") == 0 || std::strcmp(value, "120") == 0) config.halc.imuc.accel_odr = LSM6DSV16X_ODR::ODR_120Hz;
        else if (std::strcmp(value, "240Hz") == 0 || std::strcmp(value, "240") == 0) config.halc.imuc.accel_odr = LSM6DSV16X_ODR::ODR_240Hz;
        else if (std::strcmp(value, "480Hz") == 0 || std::strcmp(value, "480") == 0) config.halc.imuc.accel_odr = LSM6DSV16X_ODR::ODR_480Hz;
        else if (std::strcmp(value, "960Hz") == 0 || std::strcmp(value, "960") == 0) config.halc.imuc.accel_odr = LSM6DSV16X_ODR::ODR_960Hz;
        else if (std::strcmp(value, "1920Hz") == 0 || std::strcmp(value, "1920") == 0) config.halc.imuc.accel_odr = LSM6DSV16X_ODR::ODR_1920Hz;
        else if (std::strcmp(value, "3840Hz") == 0 || std::strcmp(value, "3840") == 0) config.halc.imuc.accel_odr = LSM6DSV16X_ODR::ODR_3840Hz;
        else if (std::strcmp(value, "7680Hz") == 0 || std::strcmp(value, "7680") == 0) config.halc.imuc.accel_odr = LSM6DSV16X_ODR::ODR_7680Hz;
        else { Serial.println("Error: Invalid accel ODR"); return; }
    } else if (std::strcmp(name, "imu_gyro_odr") == 0) {
        if (std::strcmp(value, "OFF") == 0) config.halc.imuc.gyro_odr = LSM6DSV16X_ODR::OFF;
        else if (std::strcmp(value, "1.875Hz") == 0 || std::strcmp(value, "1.875") == 0) config.halc.imuc.gyro_odr = LSM6DSV16X_ODR::ODR_1Hz875;
        else if (std::strcmp(value, "7.5Hz") == 0 || std::strcmp(value, "7.5") == 0) config.halc.imuc.gyro_odr = LSM6DSV16X_ODR::ODR_7Hz5;
        else if (std::strcmp(value, "15Hz") == 0 || std::strcmp(value, "15") == 0) config.halc.imuc.gyro_odr = LSM6DSV16X_ODR::ODR_15Hz;
        else if (std::strcmp(value, "30Hz") == 0 || std::strcmp(value, "30") == 0) config.halc.imuc.gyro_odr = LSM6DSV16X_ODR::ODR_30Hz;
        else if (std::strcmp(value, "60Hz") == 0 || std::strcmp(value, "60") == 0) config.halc.imuc.gyro_odr = LSM6DSV16X_ODR::ODR_60Hz;
        else if (std::strcmp(value, "120Hz") == 0 || std::strcmp(value, "120") == 0) config.halc.imuc.gyro_odr = LSM6DSV16X_ODR::ODR_120Hz;
        else if (std::strcmp(value, "240Hz") == 0 || std::strcmp(value, "240") == 0) config.halc.imuc.gyro_odr = LSM6DSV16X_ODR::ODR_240Hz;
        else if (std::strcmp(value, "480Hz") == 0 || std::strcmp(value, "480") == 0) config.halc.imuc.gyro_odr = LSM6DSV16X_ODR::ODR_480Hz;
        else if (std::strcmp(value, "960Hz") == 0 || std::strcmp(value, "960") == 0) config.halc.imuc.gyro_odr = LSM6DSV16X_ODR::ODR_960Hz;
        else if (std::strcmp(value, "1920Hz") == 0 || std::strcmp(value, "1920") == 0) config.halc.imuc.gyro_odr = LSM6DSV16X_ODR::ODR_1920Hz;
        else if (std::strcmp(value, "3840Hz") == 0 || std::strcmp(value, "3840") == 0) config.halc.imuc.gyro_odr = LSM6DSV16X_ODR::ODR_3840Hz;
        else if (std::strcmp(value, "7680Hz") == 0 || std::strcmp(value, "7680") == 0) config.halc.imuc.gyro_odr = LSM6DSV16X_ODR::ODR_7680Hz;
        else { Serial.println("Error: Invalid gyro ODR"); return; }
    } else if (std::strcmp(name, "imu_accel_bias_x_mps2") == 0) {
        config.halc.imuc.accel_bias_x_mps2 = static_cast<float>(std::strtod(value, nullptr));
    } else if (std::strcmp(name, "imu_accel_bias_y_mps2") == 0) {
        config.halc.imuc.accel_bias_y_mps2 = static_cast<float>(std::strtod(value, nullptr));
    } else if (std::strcmp(name, "imu_accel_bias_z_mps2") == 0) {
        config.halc.imuc.accel_bias_z_mps2 = static_cast<float>(std::strtod(value, nullptr));
    } else if (std::strcmp(name, "imu_gyro_bias_x_radps") == 0) {
        config.halc.imuc.gyro_bias_x_radps = static_cast<float>(std::strtod(value, nullptr));
    } else if (std::strcmp(name, "imu_gyro_bias_y_radps") == 0) {
        config.halc.imuc.gyro_bias_y_radps = static_cast<float>(std::strtod(value, nullptr));
    } else if (std::strcmp(name, "imu_gyro_bias_z_radps") == 0) {
        config.halc.imuc.gyro_bias_z_radps = static_cast<float>(std::strtod(value, nullptr));
    } else if (std::strcmp(name, "ser_s1_pin") == 0) {
        config.halc.serc.s1_pin = static_cast<uint8_t>(std::strtol(value, nullptr, 10));
    } else if (std::strcmp(name, "ser_s2_pin") == 0) {
        config.halc.serc.s2_pin = static_cast<uint8_t>(std::strtol(value, nullptr, 10));
    } else if (std::strcmp(name, "ser_s3_pin") == 0) {
        config.halc.serc.s3_pin = static_cast<uint8_t>(std::strtol(value, nullptr, 10));
    } else if (std::strcmp(name, "ser_s4_pin") == 0) {
        config.halc.serc.s4_pin = static_cast<uint8_t>(std::strtol(value, nullptr, 10));
    } else if (std::strcmp(name, "ser_min_us") == 0) {
        config.halc.serc.min_us = static_cast<uint16_t>(std::strtol(value, nullptr, 10));
    } else if (std::strcmp(name, "ser_max_us") == 0) {
        config.halc.serc.max_us = static_cast<uint16_t>(std::strtol(value, nullptr, 10));
    } else if (std::strcmp(name, "led_pin") == 0) {
        config.halc.ledc.pin = static_cast<uint8_t>(std::strtol(value, nullptr, 10));
    } else if (std::strcmp(name, "rpi_enabled") == 0) {
        config.halc.rpic.enabled = (std::strtol(value, nullptr, 10) != 0);
    } else if (std::strcmp(name, "rpi_uart_id") == 0) {
        config.halc.rpic.uart_id = static_cast<uint8_t>(std::strtol(value, nullptr, 10));
    } else if (std::strcmp(name, "rpi_baudrate") == 0) {
        config.halc.rpic.baudrate = static_cast<uint32_t>(std::strtol(value, nullptr, 10));
    } else if (std::strcmp(name, "rpi_tx_pin") == 0) {
        config.halc.rpic.tx_pin = static_cast<uint8_t>(std::strtol(value, nullptr, 10));
    } else if (std::strcmp(name, "rpi_rx_pin") == 0) {
        config.halc.rpic.rx_pin = static_cast<uint8_t>(std::strtol(value, nullptr, 10));
    } else if (std::strcmp(name, "rpi_rate_divisor") == 0) {
        config.halc.rpic.rate_divisor = static_cast<uint8_t>(std::strtol(value, nullptr, 10));
    } else if (std::strcmp(name, "debug_enabled") == 0) {
        config.halc.debugc.enabled = (std::strtol(value, nullptr, 10) != 0);
    } else if (std::strcmp(name, "debug_decimation") == 0) {
        config.halc.debugc.decimation = static_cast<uint32_t>(std::strtol(value, nullptr, 10));
    } else if (std::strcmp(name, "blink_hz_disarmed") == 0) {
        config.gncc.allocc.blink_hz_disarmed = static_cast<float>(std::strtod(value, nullptr));
    } else if (std::strcmp(name, "blink_hz_rate") == 0) {
        config.gncc.allocc.blink_hz_rate = static_cast<float>(std::strtod(value, nullptr));
    } else if (std::strcmp(name, "blink_hz_angle") == 0) {
        config.gncc.allocc.blink_hz_angle = static_cast<float>(std::strtod(value, nullptr));
    } else if (std::strcmp(name, "gnc_looprate_hz") == 0) {
        config.gncc.looprate_hz = static_cast<uint32_t>(std::strtol(value, nullptr, 10));
    } else if (std::strcmp(name, "angle_loop_hz") == 0) {
        config.gncc.ctlc.angle_loop_hz = static_cast<uint32_t>(std::strtol(value, nullptr, 10));
    } else if (std::strcmp(name, "roll_rate_kp") == 0) {
        config.gncc.ctlc.rate.roll.kp = static_cast<float>(std::strtod(value, nullptr));
    } else if (std::strcmp(name, "roll_rate_ki") == 0) {
        config.gncc.ctlc.rate.roll.ki = static_cast<float>(std::strtod(value, nullptr));
    } else if (std::strcmp(name, "roll_rate_kd") == 0) {
        config.gncc.ctlc.rate.roll.kd = static_cast<float>(std::strtod(value, nullptr));
    } else if (std::strcmp(name, "roll_rate_imax") == 0) {
        config.gncc.ctlc.rate.roll.i_max = static_cast<float>(std::strtod(value, nullptr));
    } else if (std::strcmp(name, "pitch_rate_kp") == 0) {
        config.gncc.ctlc.rate.pitch.kp = static_cast<float>(std::strtod(value, nullptr));
    } else if (std::strcmp(name, "pitch_rate_ki") == 0) {
        config.gncc.ctlc.rate.pitch.ki = static_cast<float>(std::strtod(value, nullptr));
    } else if (std::strcmp(name, "pitch_rate_kd") == 0) {
        config.gncc.ctlc.rate.pitch.kd = static_cast<float>(std::strtod(value, nullptr));
    } else if (std::strcmp(name, "pitch_rate_imax") == 0) {
        config.gncc.ctlc.rate.pitch.i_max = static_cast<float>(std::strtod(value, nullptr));
    } else if (std::strcmp(name, "yaw_rate_kp") == 0) {
        config.gncc.ctlc.rate.yaw.kp = static_cast<float>(std::strtod(value, nullptr));
    } else if (std::strcmp(name, "yaw_rate_ki") == 0) {
        config.gncc.ctlc.rate.yaw.ki = static_cast<float>(std::strtod(value, nullptr));
    } else if (std::strcmp(name, "yaw_rate_kd") == 0) {
        config.gncc.ctlc.rate.yaw.kd = static_cast<float>(std::strtod(value, nullptr));
    } else if (std::strcmp(name, "yaw_rate_imax") == 0) {
        config.gncc.ctlc.rate.yaw.i_max = static_cast<float>(std::strtod(value, nullptr));
    } else if (std::strcmp(name, "roll_ang_kp") == 0) {
        config.gncc.ctlc.angle.roll.kp = static_cast<float>(std::strtod(value, nullptr));
    } else if (std::strcmp(name, "pitch_ang_kp") == 0) {
        config.gncc.ctlc.angle.pitch.kp = static_cast<float>(std::strtod(value, nullptr));
    } else if (std::strcmp(name, "yaw_ang_kp") == 0) {
        config.gncc.ctlc.angle.yaw.kp = static_cast<float>(std::strtod(value, nullptr));
    } else if (std::strcmp(name, "gnc_nav_accel_bias_x") == 0) {
        config.gncc.navc.accel_bias.x() = static_cast<float>(std::strtod(value, nullptr));
    } else if (std::strcmp(name, "gnc_nav_accel_bias_y") == 0) {
        config.gncc.navc.accel_bias.y() = static_cast<float>(std::strtod(value, nullptr));
    } else if (std::strcmp(name, "gnc_nav_accel_bias_z") == 0) {
        config.gncc.navc.accel_bias.z() = static_cast<float>(std::strtod(value, nullptr));
    } else if (std::strcmp(name, "gnc_nav_gyro_bias_x") == 0) {
        config.gncc.navc.gyro_bias.x() = static_cast<float>(std::strtod(value, nullptr));
    } else if (std::strcmp(name, "gnc_nav_gyro_bias_y") == 0) {
        config.gncc.navc.gyro_bias.y() = static_cast<float>(std::strtod(value, nullptr));
    } else if (std::strcmp(name, "gnc_nav_gyro_bias_z") == 0) {
        config.gncc.navc.gyro_bias.z() = static_cast<float>(std::strtod(value, nullptr));
    } else {
        Serial.println("Error: Unknown parameter");
        return;
    }
    Serial.println("Parameter updated successfully in RAM");
}

void PARAMS::run_cli(MASTERc& config) {
    if (is_calibrating) {
        HALb bus = hal_ptr->get_bus();
        sum_ax += bus.imub.accel_body_mps2.x() + config.halc.imuc.accel_bias_x_mps2;
        sum_ay += bus.imub.accel_body_mps2.y() + config.halc.imuc.accel_bias_y_mps2;
        sum_az += bus.imub.accel_body_mps2.z() + config.halc.imuc.accel_bias_z_mps2;
        sum_gx += bus.imub.omega_body_radps.x() + config.halc.imuc.gyro_bias_x_radps;
        sum_gy += bus.imub.omega_body_radps.y() + config.halc.imuc.gyro_bias_y_radps;
        sum_gz += bus.imub.omega_body_radps.z() + config.halc.imuc.gyro_bias_z_radps;
        calibration_counter++;
        if (calibration_counter >= 200) {
            is_calibrating = false;
            float avg_ax = sum_ax / 200.0f;
            float avg_ay = sum_ay / 200.0f;
            float avg_az = sum_az / 200.0f;
            float avg_gx = sum_gx / 200.0f;
            float avg_gy = sum_gy / 200.0f;
            float avg_gz = sum_gz / 200.0f;
            float g = 9.80665f;
            float diff_x = std::abs(std::abs(avg_ax) - g);
            float diff_y = std::abs(std::abs(avg_ay) - g);
            float diff_z = std::abs(std::abs(avg_az) - g);
            float bias_ax = avg_ax;
            float bias_ay = avg_ay;
            float bias_az = avg_az;
            float min_diff = diff_x;
            if (diff_y < min_diff) {
                min_diff = diff_y;
            }
            if (diff_z < min_diff) {
                min_diff = diff_z;
            }
            if (min_diff == diff_x) {
                bias_ax = avg_ax > 0.0f ? (avg_ax - g) : (avg_ax + g);
            } else if (min_diff == diff_y) {
                bias_ay = avg_ay > 0.0f ? (avg_ay - g) : (avg_ay + g);
            } else {
                bias_az = avg_az > 0.0f ? (avg_az - g) : (avg_az + g);
            }
            config.halc.imuc.accel_bias_x_mps2 = bias_ax;
            config.halc.imuc.accel_bias_y_mps2 = bias_ay;
            config.halc.imuc.accel_bias_z_mps2 = bias_az;
            config.halc.imuc.gyro_bias_x_radps = avg_gx;
            config.halc.imuc.gyro_bias_y_radps = avg_gy;
            config.halc.imuc.gyro_bias_z_radps = avg_gz;
            Serial.println("Calibration completed successfully! Saving and rebooting...");
            save(config);
            delay(100);
            multicore_reset_core1();
            rp2040.reboot();
        }
    }

    static char buf[128];
    static size_t idx = 0;

    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\r' || c == '\n') {
            if (idx > 0) {
                buf[idx] = '\0';
                
                char* cmd = std::strtok(buf, " ");
                if (cmd != nullptr) {
                    if (std::strcmp(cmd, "help") == 0) {
                        print_help();
                    } else if (std::strcmp(cmd, "dump") == 0) {
                        print_all(config);
                    } else if (std::strcmp(cmd, "defaults") == 0) {
                        load_default_config(config);
                        Serial.println("Loaded defaults in RAM. Write 'save' to commit to flash.");
                    } else if (std::strcmp(cmd, "reboot") == 0) {
                        Serial.println("Rebooting FC...");
                        delay(100);
                        multicore_reset_core1();
                        rp2040.reboot();
                    } else if (std::strcmp(cmd, "save") == 0) {
                        save(config);
                        Serial.println("Saved config to flash! Rebooting FC...");
                        delay(100);
                        multicore_reset_core1();
                        rp2040.reboot();
                    } else if (std::strcmp(cmd, "calibrate") == 0) {
                        Serial.println("Starting IMU calibration... Keep the board flat and still.");
                        sum_ax = 0.0f;
                        sum_ay = 0.0f;
                        sum_az = 0.0f;
                        sum_gx = 0.0f;
                        sum_gy = 0.0f;
                        sum_gz = 0.0f;
                        calibration_counter = 0;
                        is_calibrating = true;
                    } else if (std::strcmp(cmd, "get") == 0) {
                        char* param_name = std::strtok(nullptr, " ");
                        if (param_name != nullptr) {
                            get_parameter(config, param_name);
                        } else {
                            Serial.println("Error: Missing parameter name");
                        }
                    } else if (std::strcmp(cmd, "set") == 0) {
                        char* param_name = std::strtok(nullptr, " =");
                        char* param_val = std::strtok(nullptr, " =");
                        if (param_name != nullptr && param_val != nullptr) {
                            set_parameter(config, param_name, param_val);
                        } else {
                            Serial.println("Error: Invalid set syntax. Use: set <param> = <value>");
                        }
                    } else {
                        Serial.println("Error: Unknown command. Type 'help' for options.");
                    }
                }
                idx = 0;
            }
        } else if (idx < sizeof(buf) - 1) {
            buf[idx++] = c;
        }
    }
}

