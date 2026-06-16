#include "hardware/hardware_util.hpp"
#include "vehicle_config/vehicle_config.hpp"

// Instantiate actuator struct
act_cmd act_cmd_struct;

// build GNC object
gnc fsw(spacey_config);

// build loop regulator
hardware_util::enforce_looprate loop_regulator(looprate_hz);

void setup() {}

void loop() {
  loop_regulator.ping(); // start loop timer

  // get inputs from hardware (dummy for now)
  gnc_util::vec imu_raw_degps;
  gnc_util::vec rate_cmd_radps;
  double thr_frac;

  // execute flight code
  act_cmd_struct = fsw.query(imu_raw_degps, rate_cmd_radps, thr_frac);

  loop_regulator.pong_and_wait(); // regulate looprate
}