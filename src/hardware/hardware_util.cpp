#include "hardware_util.hpp"
#include "../gnc/gnc_util/gnc_util.hpp"
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

double raw_rc_2_rate_degps(int rc_raw, double max_rate_degps) {
  double unclipped_frac = ((double)(rc_raw)-1490.0) / 1021.0 * 2.0;
  return gnc_util::double_clip(unclipped_frac, -1.0, 1.0) * max_rate_degps;
}

double raw_thr_2_thr_frac(int raw_thr) {
  return gnc_util::double_clip(((double)(raw_thr)-990.0) / 1021, 0.0, 1.0);
}

uint16_t thr_frac_2_DSHOT_int(double thr_frac) {
  return (uint16_t)(
      gnc_util::double_clip(thr_frac * 1999.0 + 47.0, 48, 2047.0));
}

} // namespace hardware_util