#include "RCRX.hpp"
#include <algorithm>

RCRX::RCRX(RCRXc rcrxc, uint32_t looprate_hz) 
    : rcrxc(rcrxc), 
      dt_s(1.0f / static_cast<float>(looprate_hz)), 
      telemetry_dt_s(1.0f / rcrxc.telemetry_hz), 
      time_accumulator_s(0.0f),
      is_initialized(false) {
}

RCRX::~RCRX() {}

RCRXb RCRX::update(const HALb& halb) {
    RCRXb rcrxb;
    rcrxb.arm_frac = 0.0f;
    rcrxb.mode_frac = 0.0f;
    rcrxb.thr_frac = 0.0f;
    rcrxb.roll_frac = 0.0f;
    rcrxb.pitch_frac = 0.0f;
    rcrxb.yaw_frac = 0.0f;

    if (!is_initialized) {
        if (rcrxc.uart_id == 1) {
            Serial1.setTX(0);
            Serial1.setRX(1);
            Serial1.begin(420000);
            gpio_pull_up(1); // Fix RP2350-E9 erratum: disable pull-down and enable pull-up on RX pin
            port = &Serial1;
        } else if (rcrxc.uart_id == 2) {
            // Serial2 defaults or custom pins if needed
            Serial2.begin(420000);
            port = &Serial2;
        }

        if (port != nullptr) {
            // Flush any boot-time noise or startup garbage
            while (port->available() > 0) {
                port->read();
            }
            crsf.begin(*port);
        }
        is_initialized = true;
    }

    if (port == nullptr) {
        return rcrxb;
    }

    crsf.update();

    if (!crsf.isLinkUp()) {
        return rcrxb;
    }

    time_accumulator_s += dt_s;
    if (time_accumulator_s >= telemetry_dt_s) {
        time_accumulator_s = 0.0f;
        crsf_sensor_battery_t battery_payload;
        battery_payload.voltage = htobe16(static_cast<uint16_t>(halb.vbat_volts * 10.0f));
        battery_payload.current = htobe16(0);
        battery_payload.capacity = 0;
        battery_payload.remaining = 0;
        crsf.writePacket(CRSF_ADDRESS_CRSF_RECEIVER, CRSF_FRAMETYPE_BATTERY_SENSOR, &battery_payload, sizeof(battery_payload));
    }

    float roll_us = static_cast<float>(crsf.getChannel(rcrxc.roll_ch));
    float pitch_us = static_cast<float>(crsf.getChannel(rcrxc.pitch_ch));
    float thr_us = static_cast<float>(crsf.getChannel(rcrxc.thr_ch));
    float yaw_us = static_cast<float>(crsf.getChannel(rcrxc.yaw_ch));
    float arm_us = static_cast<float>(crsf.getChannel(rcrxc.arm_ch));
    float mode_us = static_cast<float>(crsf.getChannel(rcrxc.mode_ch));

    rcrxb.roll_frac = std::clamp((roll_us - 1500.0f) / 500.0f, -1.0f, 1.0f);
    rcrxb.pitch_frac = -std::clamp((pitch_us - 1500.0f) / 500.0f, -1.0f, 1.0f);
    rcrxb.yaw_frac = std::clamp((yaw_us - 1500.0f) / 500.0f, -1.0f, 1.0f);

    rcrxb.thr_frac = std::clamp((thr_us - 1000.0f) / 1000.0f, 0.0f, 1.0f);
    rcrxb.arm_frac = std::clamp((arm_us - 1000.0f) / 1000.0f, 0.0f, 1.0f);
    rcrxb.mode_frac = std::clamp((mode_us - 1000.0f) / 1000.0f, 0.0f, 1.0f);

    return rcrxb;
}
