#define USE_ALT_LOOP_REGULATION

#include <Arduino.h>
#include "PARAMS/PARAMS.hpp"
#include "PARAMS/defaults.hpp"
#include "CFG_APP/CFG_APP.hpp"
#include "HAL/HAL.hpp"
#include "GNC/GNC.hpp"
#include "hardware/vreg.h"
#include "hardware/clocks.h"



PARAMS* params_ptr = nullptr;
CFG_APP* cfg_app_ptr = nullptr;
HAL* hal_ptr = nullptr;
GNC* gnc_ptr = nullptr;
MASTERc config_data;

static ALLb allb_km1{};
static ALLb allb_k{};
static volatile bool gnc_done = true;
static volatile bool shifted = false;
volatile bool system_ready = false;

void setup() {
    // Delay 1 seconds on boot to let the ELRS receiver and power rails fully stabilize
    delay(1000);

    #if F_CPU >= 240000000L
    vreg_set_voltage(VREG_VOLTAGE_1_20);
    delay(10);
    set_sys_clock_khz(F_CPU / 1000, true);
    #elif F_CPU >= 200000000L
    vreg_set_voltage(VREG_VOLTAGE_1_15);
    delay(10);
    set_sys_clock_khz(F_CPU / 1000, true);
    #endif

    Serial.begin(115200);
    Serial.ignoreFlowControl(true);
    delay(500);

    Serial.printf("\n--- Chesapeake Flight Controller Boot ---\n");
    Serial.printf("CPU Frequency: %lu MHz\n", rp2040.f_cpu() / 1000000UL);
    Serial.printf("Core Voltage (approx): %s\n", 
                  (F_CPU >= 240000000L) ? "1.20V" : (F_CPU >= 200000000L ? "1.15V" : "1.10V (stock)"));

    params_ptr = new PARAMS();
    if (!params_ptr->load(config_data)) {
        load_default_config(config_data);
        params_ptr->save(config_data);
    }

    cfg_app_ptr = new CFG_APP(config_data);
    hal_ptr = new HAL(config_data.halc, config_data.gncc.looprate_hz);
    gnc_ptr = new GNC(config_data.gncc);

    allb_km1.halb = hal_ptr->update(allb_km1);
    allb_km1.cfg_appb = cfg_app_ptr->update(allb_km1, config_data, *params_ptr);
    allb_k.halb = allb_km1.halb;
    allb_k.cfg_appb = allb_km1.cfg_appb;

    gnc_done = false;
    shifted = false;
    system_ready = true;
}

void loop() {
    uint32_t start_time_us = micros();

    allb_k.cfg_appb = cfg_app_ptr->update(allb_km1, config_data, *params_ptr);

    uint32_t hal_start = micros();
    allb_k.halb = hal_ptr->update(allb_km1);
    allb_k.halb.execution_time_ms = (micros() - hal_start) / 1000.0f;

    uint32_t looprate_us = 1000000UL / config_data.gncc.looprate_hz;

#ifdef USE_ALT_LOOP_REGULATION
    while (!gnc_done) {
        yield();
    }
    uint32_t elapsed_time_us = micros() - start_time_us;
    if (elapsed_time_us < looprate_us) {
        delayMicroseconds(looprate_us - elapsed_time_us);
    }
#else
    uint32_t elapsed_time_us = micros() - start_time_us;
    while (elapsed_time_us < looprate_us || !gnc_done) {
        elapsed_time_us = micros() - start_time_us;
        yield();
    }
#endif

    static bool is_calibrating = false;
    static int calibration_samples = 0;
    static Eigen::Vector3f gyro_sum(0.0f, 0.0f, 0.0f);
    static Eigen::Vector3f accel_sum(0.0f, 0.0f, 0.0f);

    if (allb_k.cfg_appb.calibrate_requested && !is_calibrating) {
        cfg_app_ptr->clear_calibrate_request();
        allb_k.cfg_appb.calibrate_requested = false;
        is_calibrating = true;
        calibration_samples = 0;
        gyro_sum = Eigen::Vector3f::Zero();
        accel_sum = Eigen::Vector3f::Zero();
        Serial.println("Starting calibration loop... Keep the board flat and still.");
    }

    if (is_calibrating) {
        gyro_sum += allb_k.halb.imub.omega_body_radps;
        accel_sum += allb_k.halb.imub.accel_body_mps2;
        calibration_samples++;

        allb_k.cfg_appb.is_calibrating = true;
        allb_k.cfg_appb.calibration_progress_frac = static_cast<float>(calibration_samples) / 500.0f;

        if (calibration_samples >= 500) {
            is_calibrating = false;
            
            Eigen::Vector3f gyro_bias = gyro_sum / 500.0f;
            Eigen::Vector3f accel_raw_avg = accel_sum / 500.0f;
            
            Eigen::Vector3f bias_sensor = accel_raw_avg - accel_raw_avg.normalized() * 9.80665f;
            Eigen::Vector3f accel_bias = config_data.gncc.navc.q_IMU2body.conjugate() * bias_sensor;

            config_data.gncc.navc.imu_calc.gyro_bias = gyro_bias;
            config_data.gncc.navc.imu_calc.accel_bias = accel_bias;

            params_ptr->save(config_data);

            Serial.println("Calibration complete! Saved biases to LittleFS config file:");
            Serial.printf("  Gyro Biases:  x=%.6f, y=%.6f, z=%.6f\n", gyro_bias.x(), gyro_bias.y(), gyro_bias.z());
            Serial.printf("  Accel Biases: x=%.6f, y=%.6f, z=%.6f\n", accel_bias.x(), accel_bias.y(), accel_bias.z());

            allb_k.cfg_appb.is_calibrating = false;
            allb_k.cfg_appb.calibration_progress_frac = 1.0f;
        }
    }

    if (gnc_done && !shifted) {
        allb_km1 = allb_k;
        shifted = true;
    }

    if (allb_k.cfg_appb.defaults_requested) {
        cfg_app_ptr->clear_defaults_request();
        allb_k.cfg_appb.defaults_requested = false;
        load_default_config(config_data);
    }

    if (allb_k.cfg_appb.save_requested) {
        cfg_app_ptr->clear_save_request();
        allb_k.cfg_appb.save_requested = false;
        params_ptr->save(config_data);
        delay(100);
        multicore_reset_core1();
        rp2040.reboot();
    }

    if (allb_k.cfg_appb.reboot_requested) {
        cfg_app_ptr->clear_reboot_request();
        allb_k.cfg_appb.reboot_requested = false;
        delay(100);
        multicore_reset_core1();
        rp2040.reboot();
    }

    shifted = false;
    gnc_done = false;
    yield();
}

void __not_in_flash_func(core1_halt_loop)() {
    while (!system_ready) {
        asm volatile("nop");
    }
}

void setup1() {
    delay(1000);
}

void loop1() {
    if (!system_ready) {
        core1_halt_loop();
        return;
    }

    if (!gnc_done) {
        gnc_ptr->update_dual_core(allb_k, allb_km1);
        gnc_done = true;
    }
    yield();
}


