#include "BAT.hpp"
#include <Arduino.h>

BAT::BAT(BATc cfg) : batc(cfg) {
    analogReadResolution(12);
}

BAT::~BAT() {}

float BAT::update(const HALb& halb) {
    float raw_reading = static_cast<float>(analogRead(batc.pin));
    float pin_voltage_volts = (raw_reading / 4095.0f) * 3.3f;
    float battery_voltage_volts = pin_voltage_volts * batc.division_factor;
    return battery_voltage_volts;
}
