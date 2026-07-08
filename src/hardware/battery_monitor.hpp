#pragma once
#include <Arduino.h>

namespace hardware_util {

class battery_monitor {
public:
  battery_monitor(int pin, double multiplier);
  ~battery_monitor();
  
  void begin();
  double read_voltage();

private:
  int adc_pin;
  double voltage_multiplier;
};

} // namespace hardware_util
