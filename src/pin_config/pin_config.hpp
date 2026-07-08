#pragma once
#include <Arduino.h>

struct pin_cfg_t {
  int elrs_rx_pin;
  int elrs_tx_pin;
  int esc_1_pin;
  int esc_2_pin;
  int servo_1_pin;
  int servo_2_pin;
  int servo_3_pin;
  int servo_4_pin;
  int battery_pin;
  int cs_pin;
  int imu_sck_pin;
  int imu_mosi_pin;
  int imu_miso_pin;
};

extern pin_cfg_t pin_config;

// Backward compatibility with legacy pin variables
extern int ELRS_RX_PIN;
extern int ELRS_TX_PIN;
extern int ESC_1_PIN;
extern int ESC_2_PIN;
extern int SERVO_1_PIN;
extern int SERVO_2_PIN;
extern int SERVO_3_PIN;
extern int SERVO_4_PIN;
extern int BATTERY_PIN;
extern int CS_PIN;
extern int IMU_SCK_PIN;
extern int IMU_MOSI_PIN;
extern int IMU_MISO_PIN;

void pin_config_load_defaults();
void pin_config_sync_legacy();
bool pin_config_set(const char* name, int value);
bool pin_config_get(const char* name, int& value);
void pin_config_print_all();
