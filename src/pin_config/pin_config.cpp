#include "pin_config.hpp"
#include <string.h>

pin_cfg_t pin_config;

// Define legacy global pin variables
int ELRS_RX_PIN = D7;
int ELRS_TX_PIN = D6;
int ESC_1_PIN = D1;
int ESC_2_PIN = D2;
int SERVO_1_PIN = D5;
int SERVO_2_PIN = D8;
int SERVO_3_PIN = D9;
int SERVO_4_PIN = D10;
int CS_PIN = 9;
int IMU_SCK_PIN = 10;
int IMU_MOSI_PIN = 11;
int IMU_MISO_PIN = 12;

struct PinParam {
  const char* name;
  int* value_ptr;
};

// Map parameter names to struct members
static PinParam pin_params[] = {
  {"elrs_rx", &pin_config.elrs_rx_pin},
  {"elrs_tx", &pin_config.elrs_tx_pin},
  {"esc_1", &pin_config.esc_1_pin},
  {"esc_2", &pin_config.esc_2_pin},
  {"servo_1", &pin_config.servo_1_pin},
  {"servo_2", &pin_config.servo_2_pin},
  {"servo_3", &pin_config.servo_3_pin},
  {"servo_4", &pin_config.servo_4_pin},
  {"cs", &pin_config.cs_pin},
  {"imu_sck", &pin_config.imu_sck_pin},
  {"imu_mosi", &pin_config.imu_mosi_pin},
  {"imu_miso", &pin_config.imu_miso_pin}
};

static const size_t num_pin_params = sizeof(pin_params) / sizeof(PinParam);

void pin_config_load_defaults() {
  pin_config.elrs_rx_pin = D7;
  pin_config.elrs_tx_pin = D6;
  pin_config.esc_1_pin = D1;
  pin_config.esc_2_pin = D2;
  pin_config.servo_1_pin = D5;
  pin_config.servo_2_pin = D8;
  pin_config.servo_3_pin = D9;
  pin_config.servo_4_pin = D10;
  pin_config.cs_pin = 9;
  pin_config.imu_sck_pin = 10;
  pin_config.imu_mosi_pin = 11;
  pin_config.imu_miso_pin = 12;
  pin_config_sync_legacy();
}

void pin_config_sync_legacy() {
  ELRS_RX_PIN = pin_config.elrs_rx_pin;
  ELRS_TX_PIN = pin_config.elrs_tx_pin;
  ESC_1_PIN = pin_config.esc_1_pin;
  ESC_2_PIN = pin_config.esc_2_pin;
  SERVO_1_PIN = pin_config.servo_1_pin;
  SERVO_2_PIN = pin_config.servo_2_pin;
  SERVO_3_PIN = pin_config.servo_3_pin;
  SERVO_4_PIN = pin_config.servo_4_pin;
  CS_PIN = pin_config.cs_pin;
  IMU_SCK_PIN = pin_config.imu_sck_pin;
  IMU_MOSI_PIN = pin_config.imu_mosi_pin;
  IMU_MISO_PIN = pin_config.imu_miso_pin;
}

bool pin_config_set(const char* name, int value) {
  for (size_t i = 0; i < num_pin_params; ++i) {
    if (strcasecmp(name, pin_params[i].name) == 0) {
      *(pin_params[i].value_ptr) = value;
      pin_config_sync_legacy();
      return true;
    }
  }
  return false;
}

bool pin_config_get(const char* name, int& value) {
  for (size_t i = 0; i < num_pin_params; ++i) {
    if (strcasecmp(name, pin_params[i].name) == 0) {
      value = *(pin_params[i].value_ptr);
      return true;
    }
  }
  return false;
}

void pin_config_print_all() {
  for (size_t i = 0; i < num_pin_params; ++i) {
    Serial.print("set pin_");
    Serial.print(pin_params[i].name);
    Serial.print(" = ");
    Serial.println(*(pin_params[i].value_ptr));
  }
}
