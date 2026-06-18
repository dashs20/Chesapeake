#pragma once
#include <Arduino.h>

namespace hardware_util {

class enforce_looprate {
public:
  enforce_looprate(double looprate_hz); // Default Constructor
  ~enforce_looprate();                  // Destructor
  void ping();                          // Start timer
  void pong_and_wait();                 // Stop timer and wait

private:
  unsigned long des_looptime_micro_s;
  unsigned long start_time_micro_s;
};

double raw_rc_2_rate_degps(int rc_raw, double max_rate_degps);

double raw_thr_2_thr_frac(int raw_thr);

uint16_t thr_frac_2_DSHOT_int(double thr_frac);

} // namespace hardware_util
