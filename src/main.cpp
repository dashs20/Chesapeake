#define USE_ALT_LOOP_REGULATION

#include <Arduino.h>
#include "PARAMS/PARAMS.hpp"
#include "PARAMS/defaults.hpp"
#include "CFG_APP/CFG_APP.hpp"
#include "HAL/HAL.hpp"
#include "GNC/GNC.hpp"

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
    Serial.begin(115200);
    delay(500);

    params_ptr = new PARAMS();
    if (!params_ptr->load(config_data)) {
        load_default_config(config_data);
        params_ptr->save(config_data);
    }

    cfg_app_ptr = new CFG_APP();
    hal_ptr = new HAL(config_data.halc);
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

    if (gnc_done && !shifted) {
        allb_km1 = allb_k;
        shifted = true;
    }

    if (allb_k.gncb.cal_feedback.calibration_done) {
        cfg_app_ptr->clear_calibrate_request();
        allb_k.cfg_appb.calibrate_requested = false;
        config_data.halc.imuc.accel_bias_x_mps2 += allb_k.gncb.cal_feedback.accel_bias_x;
        config_data.halc.imuc.accel_bias_y_mps2 += allb_k.gncb.cal_feedback.accel_bias_y;
        config_data.halc.imuc.accel_bias_z_mps2 += allb_k.gncb.cal_feedback.accel_bias_z;
        config_data.halc.imuc.gyro_bias_x_radps += allb_k.gncb.cal_feedback.gyro_bias_x;
        config_data.halc.imuc.gyro_bias_y_radps += allb_k.gncb.cal_feedback.gyro_bias_y;
        config_data.halc.imuc.gyro_bias_z_radps += allb_k.gncb.cal_feedback.gyro_bias_z;
        params_ptr->save(config_data);
        delay(100);
        multicore_reset_core1();
        rp2040.reboot();
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


