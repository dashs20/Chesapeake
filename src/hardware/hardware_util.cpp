#include "hardware_util.hpp"
#include <Arduino.h>

namespace hardware_util {

hardware_util::enforce_looprate::enforce_looprate(double looprate_hz) {
  des_looptime_micro_s = (unsigned long)(1000000.0 / looprate_hz);
  start_time_micro_s = 0;
}

void hardware_util::enforce_looprate::ping() { start_time_micro_s = micros(); }
void hardware_util::enforce_looprate::pong_and_wait() {
  unsigned long looptime_micro_s = micros() - start_time_micro_s;
  if (looptime_micro_s <= des_looptime_micro_s) {
    delayMicroseconds(des_looptime_micro_s - looptime_micro_s);
  }
}

} // namespace hardware_util