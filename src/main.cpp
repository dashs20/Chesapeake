#include <Arduino.h>
#include "PARAMS/PARAMS.hpp"
#include "PARAMS/defaults.hpp"
#include "HAL/HAL.hpp"
#include "GNC/GNC.hpp"

PARAMS* params_ptr = nullptr;
HAL* hal_ptr = nullptr;
GNC* gnc_ptr = nullptr;
MASTERc config_data;

void setup() {
    Serial.begin(115200);
    // Short boot delay to ensure the USB serial stack enumerates reliably
    delay(500);

    params_ptr = new PARAMS();
    if (!params_ptr->load(config_data)) {
        load_default_config(config_data);
        params_ptr->save(config_data);
    }

    hal_ptr = new HAL(config_data.halc);
    gnc_ptr = new GNC(config_data.gncc);
}

void loop() {
    uint32_t start_time_us = micros();

    params_ptr->run_cli(config_data);

    static ACTb actb_data{};
    HALb halb_data = hal_ptr->update(actb_data);
    actb_data = gnc_ptr->update(halb_data);

    // Print real-time telemetry to Serial at 20Hz (every 50ms)
    static uint32_t last_telemetry_ms = 0;
    uint32_t now_ms = millis();
    if (now_ms - last_telemetry_ms >= 50) {
        last_telemetry_ms = now_ms;
        if (gnc_ptr != nullptr) {
            const GNCb& gnc_bus = gnc_ptr->get_bus();
            Serial.printf("$TEL,%.2f,%.4f,%.4f,%.4f,%.4f\n",
                          halb_data.vbat_volts,
                          gnc_bus.navb.q_earth2body.w(),
                          gnc_bus.navb.q_earth2body.x(),
                          gnc_bus.navb.q_earth2body.y(),
                          gnc_bus.navb.q_earth2body.z());
        }
    }

    uint32_t target_dt_us = 1000000UL / config_data.gncc.looprate_hz;
    uint32_t elapsed_time_us = micros() - start_time_us;
    if (elapsed_time_us < target_dt_us) {
        delayMicroseconds(target_dt_us - elapsed_time_us);
    }
    yield();
}
