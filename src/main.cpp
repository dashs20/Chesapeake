#include "ICM42688.h"
#include "gnc_config/gnc_config.hpp"
#include "hardware/hardware_util.hpp"
#include "pin_config/pin_config.hpp"
#include <AlfredoCrsf.h>
#include <PIO_DShot.h>
#include <Servo.h>

/*
~~~ Configure GNC ~~~
*/

// Instantiate actuator command storage struct
act_cmd act_cmd_struct;

// build GNC object
gnc fsw(spacey_config);

// build loop regulator
hardware_util::enforce_looprate loop_regulator(looprate_hz);

/*
~~~ Configure Hardware ~~~
*/

// ELRS
AlfredoCRSF elrs;
int thr_raw;
int roll_raw;
int pitch_raw;
int yaw_raw;
int arm_raw;
gnc_util::vec rate_cmd_degps;
double thr_frac;

enum class STATE {
  NOTHING = 0,
  SERVOS = 1,
  EVERYTHING = 2,
};
STATE arm_state;

// DSHOT
DShotX4 *esc;
uint16_t throttles[4] = {0};

// SERVO
Servo servo1;
Servo servo2;

// IMU
ICM42688 imu(SPI, CS_PIN);
gnc_util::vec imu_raw_degps;

void setup() {

  /*
  Hardware setup
  */

  // ELRS
  Serial1.setRX(ELRS_RX_PIN);
  Serial1.setTX(ELRS_TX_PIN);
  elrs.begin(Serial1);

  // IMU
  imu.setGyroODR(ICM42688::odr1k); // 1 kHz output rate
  imu.setGyroFS(ICM42688::dps500); // 500 deg/s range

  // SERVO
  servo1.attach(SERVO_1_PIN, 500, 2500); // duty cycle range for my servos
  servo2.attach(SERVO_2_PIN, 500, 2500);

  // DSHOT (last so we can immediately send 0 throttle until they arm)
  // Command zero throttle for 5 seconds

  uint32_t start = millis();
  while (millis() - start < 5000) {
    esc->sendThrottles(throttles);
    delay(1);
  }
}

void loop() {
  loop_regulator.ping(); // start loop timer

  /*
  Get Inputs from Hardware
  */

  // Get raw IMU reading and store in vector
  imu.getAGT();
  imu_raw_degps.x = imu.gyrX();
  imu_raw_degps.y = imu.gyrY();
  imu_raw_degps.z = imu.gyrZ();

  // Get RC command and convert to rate/throttle fraction
  roll_raw = elrs.getChannel(0);
  pitch_raw = elrs.getChannel(1);
  thr_raw = elrs.getChannel(2);
  yaw_raw = elrs.getChannel(3);
  arm_raw = elrs.getChannel(5);
  rate_cmd_degps.x =
      hardware_util::raw_rc_2_rate_degps(roll_raw, max_rate_degps);
  rate_cmd_degps.y =
      hardware_util::raw_rc_2_rate_degps(pitch_raw, max_rate_degps);
  rate_cmd_degps.z =
      hardware_util::raw_rc_2_rate_degps(yaw_raw, max_rate_degps);
  thr_frac = hardware_util::raw_thr_2_thr_frac(thr_raw);
  if (arm_raw > 900 & arm_raw < 1100) {
    arm_state = STATE::NOTHING;
  }
  if (arm_raw > 1400 & arm_raw < 1600) {
    arm_state = STATE::SERVOS;
  }
  if (arm_raw > 1900) {
    arm_state = STATE::EVERYTHING;
  }

  /*
  Execute GNC
  */
  act_cmd_struct = fsw.query(imu_raw_degps, rate_cmd_degps, thr_frac);

  /*
  Command outputs to hardware
  */

  switch (arm_state) {
  case STATE::NOTHING: // fully disarmed (switch all the way down)
    servo1.write(90.0);
    servo2.write(90.0);
    throttles[0] = hardware_util::thr_frac_2_DSHOT_int(0);
    throttles[1] = hardware_util::thr_frac_2_DSHOT_int(0);
    esc->sendThrottles(throttles);
    break;
  case STATE::SERVOS: // servos only
    servo1.write(act_cmd_struct.theta_1_deg);
    servo2.write(act_cmd_struct.theta_2_deg);
    throttles[0] = hardware_util::thr_frac_2_DSHOT_int(0);
    throttles[1] = hardware_util::thr_frac_2_DSHOT_int(0);
    esc->sendThrottles(throttles);
    break;
  case STATE::EVERYTHING: // servos + motors
    servo1.write(act_cmd_struct.theta_1_deg);
    servo2.write(act_cmd_struct.theta_2_deg);
    throttles[0] = hardware_util::thr_frac_2_DSHOT_int(act_cmd_struct.th_frac);
    throttles[1] = hardware_util::thr_frac_2_DSHOT_int(act_cmd_struct.tl_frac);
    esc->sendThrottles(throttles);
    break;
  }

  loop_regulator.pong_and_wait(); // regulate looprate
}