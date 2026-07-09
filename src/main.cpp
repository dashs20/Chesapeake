#define USE_ALT_LOOP_REGULATION

#include <Arduino.h>
#include "PARAMS/PARAMS.hpp"
#include "PARAMS/defaults.hpp"
#include "HAL/HAL.hpp"
#include "GNC/GNC.hpp"

PARAMS* params_ptr = nullptr;
HAL* hal_ptr = nullptr;
GNC* gnc_ptr = nullptr;
MASTERc config_data;

static ALLb allb_km1{};
static ALLb allb_k{};
static volatile bool gnc_done = true;
static volatile bool shifted = false;
static volatile bool system_ready = false;

void setup() {
    Serial.begin(115200);
    delay(500);

    params_ptr = new PARAMS();
    if (!params_ptr->load(config_data)) {
        load_default_config(config_data);
        params_ptr->save(config_data);
    }

    hal_ptr = new HAL(config_data.halc);
    gnc_ptr = new GNC(config_data.gncc);

    allb_km1.halb = hal_ptr->update(allb_km1);
    allb_k.halb = allb_km1.halb;

    gnc_done = false;
    shifted = false;
    system_ready = true;
}

void loop() {
    uint32_t start_time_us = micros();

    params_ptr->run_cli(config_data);

    allb_k.halb = hal_ptr->update(allb_km1);

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

    static uint32_t last_telemetry_ms = 0;
    uint32_t now_ms = millis();
    if (now_ms - last_telemetry_ms >= 50) {
        last_telemetry_ms = now_ms;
        Serial.printf("$TEL,%.2f,%.4f,%.4f,%.4f,%.4f\n",
                      allb_km1.halb.vbat_volts,
                      allb_km1.navb.q_earth2body.w(),
                      allb_km1.navb.q_earth2body.x(),
                      allb_km1.navb.q_earth2body.y(),
                      allb_km1.navb.q_earth2body.z());
    }

    shifted = false;
    gnc_done = false;
    yield();
}

void setup1() {
}

void loop1() {
    if (!system_ready) {
        yield();
        return;
    }

    if (!gnc_done) {
        gnc_ptr->update_dual_core(allb_k, allb_km1);
        gnc_done = true;
    }
    yield();
}
