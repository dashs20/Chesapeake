#include "gnc_config/gnc_config.hpp"
#include "hardware/hardware_util.hpp"
#include "pin_config/pin_config.hpp"
#include "config_manager.hpp"
#include "cli_handler.hpp"
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
gnc *fsw = nullptr;

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

// IMU pointer
LSM6DSV16XSensor *imu = nullptr;
gnc_util::vec imu_raw_degps;

void setup() {
  Serial.begin(115200);

  // Initialize and load configurations from EEPROM or defaults
  config_manager_init();

  // Instantiate fsw and loop regulator using loaded parameters
  fsw = new gnc(spacey_config);
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
  int num_motors = (spacey_config.allocator_type == AllocatorType::QUAD) ? 4 : 2;
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

  // Debug: Print rotated IMU rate
  static uint32_t last_print = 0;
  if (millis() - last_print >= 100) {
    last_print = millis();
    gnc_util::vec imu_rotated = gnc_util::euler_xyz_rotate_deg(
        imu_raw_degps, spacey_config.imu_euler_xyz_deg);
    Serial.print("Rot_X: ");
    Serial.print(imu_rotated.x);
    Serial.print(" | Rot_Y: ");
    Serial.print(imu_rotated.y);
    Serial.print(" | Rot_Z: ");
    Serial.println(imu_rotated.z);
  }

  if (!elrs.isLinkUp()) {
    arm_state = STATE::NOTHING;
  } else {
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
    if (arm_raw > 900 && arm_raw < 1100) {
      arm_state = STATE::NOTHING;
    }
    if (arm_raw > 1400 && arm_raw < 1600) {
      arm_state = STATE::SERVOS;
    }
    if (arm_raw > 1900) {
      arm_state = STATE::EVERYTHING;
    }
  }

  /*
  Execute GNC
  */
  act_cmd_struct = fsw->query(imu_raw_degps, rate_cmd_degps, thr_frac);

  /*
  Command outputs to hardware
  */

  int active_motors = (spacey_config.allocator_type == AllocatorType::QUAD) ? 4 : 2;

  switch (arm_state) {
  case STATE::NOTHING: // fully disarmed (switch all the way down)
    servo1.write(90.0);
    servo2.write(90.0);
    for (int i = 0; i < 4; i++) throttles[i] = 0;
    esc->sendThrottles(throttles);
    break;
  case STATE::SERVOS: // servos only
    servo1.write(act_cmd_struct.servos[0]);
    servo2.write(act_cmd_struct.servos[1]);
    for (int i = 0; i < 4; i++) throttles[i] = 0;
    esc->sendThrottles(throttles);
    break;
  case STATE::EVERYTHING: // servos + motors
    servo1.write(act_cmd_struct.servos[0]);
    servo2.write(act_cmd_struct.servos[1]);
    for (int i = 0; i < active_motors; i++) {
      throttles[i] = hardware_util::thr_frac_2_DSHOT_int(act_cmd_struct.motors[i]);
    }
    // Set inactive motors to 0
    for (int i = active_motors; i < 4; i++) {
      throttles[i] = 0;
    }
    esc->sendThrottles(throttles);
    break;
  }

  // Update Serial CLI
  cli_handler_update();

  loop_regulator->pong_and_wait(); // regulate looprate
}