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

} // namespace hardware_util
