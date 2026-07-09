#include "LED.hpp"
#include <Arduino.h>

LED::LED(uint8_t pin) : pin_num(pin) {
    if (pin_num != 255) {
        pinMode(pin_num, OUTPUT);
        digitalWrite(pin_num, LOW);
    }
}

LED::~LED() {}

void LED::update(const ACTb& actb) {
    if (pin_num == 255) {
        return;
    }
    float blink_hz = actb.LED_blink_Hz;
    if (blink_hz <= 0.0f) {
        digitalWrite(pin_num, LOW);
        return;
    }
    uint32_t period_ms = static_cast<uint32_t>(1000.0f / blink_hz);
    uint32_t half_period_ms = period_ms / 2;
    if (half_period_ms == 0) {
        digitalWrite(pin_num, LOW);
        return;
    }
    uint32_t current_time_ms = millis();
    bool state = (current_time_ms % period_ms) < half_period_ms;
    digitalWrite(pin_num, state ? HIGH : LOW);
}
