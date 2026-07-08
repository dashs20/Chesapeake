#include "gnc_config/gnc_config.hpp"
#include "hardware/hardware_util.hpp"
#include "pin_config/pin_config.hpp"
#include "config_manager.hpp"
#include "cli_handler.hpp"
#include "gnc/fsm.hpp"
#include <AlfredoCrsf.h>
#include <LSM6DSV16XSensor.h>
#include <PIO_DShot.h>
#include <Servo.h>


/*
~~~ Configure GNC ~~~
*/

// Instantiate actuator command storage struct
act_cmd act_cmd_struct;

// build GNC object pointer
gnc *gnc_inst = nullptr;

// build Flight State Machine
FlightVsm fsm;

// build loop regulator pointer
hardware_util::enforce_looprate *loop_regulator = nullptr;

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

// DSHOT
DShotX4 *esc;
uint16_t throttles[4] = {0};

// SERVO
Servo servo1;
Servo servo2;

// IMU pointer
LSM6DSV16XSensor *imu = nullptr;
gnc_util::vec imu_raw_degps;

void setup() {
  Serial.begin(115200);

  // Initialize and load configurations from EEPROM or defaults
  config_manager_init();

  // Initialize Flight State Machine
  fsm.begin();

  // Instantiate gnc_inst and loop regulator using loaded parameters
  gnc_inst = new gnc(chesapeake_config);
  loop_regulator = new hardware_util::enforce_looprate(looprate_hz);

  /*
  Hardware setup
  */

  // ELRS
  Serial1.setRX(ELRS_RX_PIN);
  Serial1.setTX(ELRS_TX_PIN);
  Serial1.begin(CRSF_BAUDRATE);
  elrs.begin(Serial1);

  // IMU
  SPI1.setSCK(IMU_SCK_PIN);
  SPI1.setTX(IMU_MOSI_PIN);
  SPI1.setRX(IMU_MISO_PIN);
  SPI1.begin();

  imu = new LSM6DSV16XSensor(&SPI1, CS_PIN);
  if (imu->begin() != LSM6DSV16X_OK) {
    Serial.println("LSM6DSV16X IMU initialization failed!");
  }
  imu->Enable_X();
  imu->Enable_G();
  imu->Set_X_ODR(960.0f);
  imu->Set_G_ODR(960.0f);
  imu->Set_G_FS(2000);

  // SERVO
  servo1.attach(SERVO_1_PIN, 500, 2500); // duty cycle range for my servos
  servo2.attach(SERVO_2_PIN, 500, 2500);

  // DSHOT (last so we can immediately send 0 throttle until they arm)
  // Command zero throttle for 3 seconds
  int num_motors = 4;
  esc = new DShotX4(ESC_1_PIN, num_motors, 300);
  uint32_t start = millis();
  while (millis() - start < 3000) {
    esc->sendThrottles(throttles);
    delay(1);
  }
}

void loop() {
  loop_regulator->ping(); // start loop timer

  // Update ELRS receiver state
  elrs.update();

  /*
  Get Inputs from Hardware
  */

  // Get raw IMU reading and store in vector
  int32_t gyro_raw[3];
  imu->Get_G_Axes(gyro_raw);
  imu_raw_degps.x = gyro_raw[0] / 1000.0;
  imu_raw_degps.y = gyro_raw[1] / 1000.0;
  imu_raw_degps.z = gyro_raw[2] / 1000.0;

  // Build input for State Machine
  VsmInput fsm_input = {};
  fsm_input.is_link_up = elrs.isLinkUp();
  
  if (fsm_input.is_link_up) {
    // Get RC command and convert to rate/throttle fraction
    roll_raw = elrs.getChannel(1);
    pitch_raw = elrs.getChannel(2);
    thr_raw = elrs.getChannel(3);
    yaw_raw = elrs.getChannel(4);
    arm_raw = elrs.getChannel(6);
    
    rate_cmd_degps.x =
        hardware_util::raw_rc_2_rate_degps(roll_raw, max_rate_degps);
    rate_cmd_degps.y =
        hardware_util::raw_rc_2_rate_degps(pitch_raw, max_rate_degps);
    rate_cmd_degps.z =
        hardware_util::raw_rc_2_rate_degps(yaw_raw, max_rate_degps);
    thr_frac = hardware_util::raw_thr_2_thr_frac(thr_raw);
    
    fsm_input.arm_channel_val = arm_raw;
  } else {
    // Default / safe rate/throttle values when link is lost
    rate_cmd_degps.x = 0.0;
    rate_cmd_degps.y = 0.0;
    rate_cmd_degps.z = 0.0;
    thr_frac = 0.0;
    fsm_input.arm_channel_val = 1000.0; // Force disarm channel value
  }

  // Update Flight State Machine
  fsm.update(fsm_input);

  /*
  Execute GNC
  */
  act_cmd_struct = gnc_inst->query(imu_raw_degps, rate_cmd_degps, thr_frac);

  // Filter actuator outputs through VSM safety constraints
  act_cmd filtered_cmd = fsm.filter_commands(act_cmd_struct);

  /*
  Command outputs to hardware
  */
  servo1.write(filtered_cmd.servos[0]);
  servo2.write(filtered_cmd.servos[1]);
  
  for (int i = 0; i < 4; i++) {
    throttles[i] = hardware_util::thr_frac_2_DSHOT_int(filtered_cmd.motors[i]);
  }
  esc->sendThrottles(throttles);

  // Update Serial CLI
  cli_handler_update();

  loop_regulator->pong_and_wait(); // regulate looprate
}