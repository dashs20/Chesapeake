#include "config_manager.hpp"
#include "pin_config/pin_config.hpp"
#include "gnc_config/gnc_config.hpp"
#include <EEPROM.h>
#include <Arduino.h>

#define EEPROM_SIZE 512
#define EEPROM_MAGIC 0x50414359 // "SPCY" in hex
#define EEPROM_VERSION 1

#define ADDR_MAGIC 0
#define ADDR_VERSION 4
#define ADDR_PIN_CFG 8
#define ADDR_GNC_CFG 48

void config_manager_init() {
  EEPROM.begin(EEPROM_SIZE);
  
  uint32_t magic = 0;
  uint32_t version = 0;
  EEPROM.get(ADDR_MAGIC, magic);
  EEPROM.get(ADDR_VERSION, version);
  
  if (magic == EEPROM_MAGIC && version == EEPROM_VERSION) {
    Serial.println("Loading configuration from EEPROM...");
    
    // Read pin config
    EEPROM.get(ADDR_PIN_CFG, pin_config);
    pin_config_sync_legacy();
    
    // Read GNC config
    EEPROM.get(ADDR_GNC_CFG, gnc_params);
    gnc_config_sync_legacy();
  } else {
    Serial.println("EEPROM uninitialized or version mismatch! Loading defaults...");
    config_manager_load_defaults();
    config_manager_save(); // save default config to EEPROM
  }
}

void config_manager_save() {
  Serial.println("Saving configuration to EEPROM...");
  
  uint32_t magic = EEPROM_MAGIC;
  uint32_t version = EEPROM_VERSION;
  
  EEPROM.put(ADDR_MAGIC, magic);
  EEPROM.put(ADDR_VERSION, version);
  EEPROM.put(ADDR_PIN_CFG, pin_config);
  EEPROM.put(ADDR_GNC_CFG, gnc_params);
  
  bool success = EEPROM.commit();
  if (success) {
    Serial.println("Configuration saved successfully.");
  } else {
    Serial.println("Error: EEPROM commit failed!");
  }
}

void config_manager_load_defaults() {
  pin_config_load_defaults();
  gnc_config_load_defaults();
}

void config_manager_reboot() {
  Serial.println("Rebooting flight controller...");
  Serial.flush();
  delay(100);
  rp2040.reboot();
  while (true) {}
}
