#include "battery_monitor.hpp"
#include <Arduino.h>

namespace hardware_util {

battery_monitor::battery_monitor(int pin, double multiplier)
    : adc_pin(pin), voltage_multiplier(multiplier) {}

battery_monitor::~battery_monitor() {}

void battery_monitor::begin() {
  // Set ADC resolution to 12 bits (0-4095 range) for the RP2350
  analogReadResolution(12);
}

double battery_monitor::read_voltage() {
  int raw = analogRead(adc_pin);
  // Scale raw ADC count (12-bit, 3.3V ref) to actual voltage at pin
  double pin_volts = (double)raw * 3.3 / 4095.0;
  // Convert pin voltage back to original battery voltage via multiplier
  return pin_volts * voltage_multiplier;
}

} // namespace hardware_util
