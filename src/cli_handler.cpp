#include "cli_handler.hpp"
#include "pin_config/pin_config.hpp"
#include "gnc_config/gnc_config.hpp"
#include "config_manager.hpp"
#include <Arduino.h>
#include <string.h>

#define BUFFER_SIZE 80
static char cli_buffer[BUFFER_SIZE];
static int cli_index = 0;

static void parse_and_execute(char* line) {
  // Trim leading/trailing spaces
  while (*line == ' ' || *line == '\t') line++;
  int len = strlen(line);
  while (len > 0 && (line[len - 1] == ' ' || line[len - 1] == '\t' || line[len - 1] == '\r' || line[len - 1] == '\n')) {
    line[len - 1] = '\0';
    len--;
  }
  
  if (len == 0) return;
  
  // 1. HELP
  if (strcasecmp(line, "help") == 0) {
    Serial.println("--- Spacey FSW CLI Help ---");
    Serial.println("Commands:");
    Serial.println("  help               - Show this help menu");
    Serial.println("  status             - Show current vehicle status");
    Serial.println("  dump / diff        - Dump all configuration settings");
    Serial.println("  get <parameter>    - Get a setting value");
    Serial.println("  set <param> = <val>- Set a setting value (RAM only)");
    Serial.println("  save               - Write RAM configuration to EEPROM and reboot");
    Serial.println("  defaults           - Reset config to compile-time defaults and reboot");
    Serial.println("--------------------------");
    return;
  }
  
  // 2. STATUS
  if (strcasecmp(line, "status") == 0) {
    Serial.println("--- Vehicle Status ---");
    Serial.print("Loop Rate (target): ");
    Serial.print(looprate_hz);
    Serial.println(" Hz");
    Serial.print("Max rate: ");
    Serial.print(max_rate_degps);
    Serial.println(" deg/s");
    Serial.println("----------------------");
    return;
  }
  
  // 3. DUMP / DIFF
  if (strcasecmp(line, "dump") == 0 || strcasecmp(line, "diff") == 0) {
    Serial.println("# Spacey FSW Configuration Dump");
    pin_config_print_all();
    gnc_config_print_all();
    return;
  }
  
  // 4. SAVE
  if (strcasecmp(line, "save") == 0) {
    config_manager_save();
    config_manager_reboot();
    return;
  }
  
  // 5. DEFAULTS
  if (strcasecmp(line, "defaults") == 0) {
    config_manager_load_defaults();
    config_manager_save();
    config_manager_reboot();
    return;
  }
  
  // 6. GET <parameter>
  if (strncasecmp(line, "get ", 4) == 0) {
    char* param = line + 4;
    while (*param == ' ' || *param == '\t') param++;
    
    // Check if it's a pin config
    if (strncasecmp(param, "pin_", 4) == 0) {
      char* pin_param_name = param + 4;
      int val;
      if (pin_config_get(pin_param_name, val)) {
        Serial.print("pin_");
        Serial.print(pin_param_name);
        Serial.print(" = ");
        Serial.println(val);
      } else {
        Serial.println("Error: Pin parameter not found.");
      }
    } else {
      double val;
      if (gnc_config_get(param, val)) {
        Serial.print(param);
        Serial.print(" = ");
        Serial.println(val, 6);
      } else {
        Serial.println("Error: GNC parameter not found.");
      }
    }
    return;
  }
  
  // 7. SET <parameter> = <value> or SET <parameter> <value>
  if (strncasecmp(line, "set ", 4) == 0) {
    char* args = line + 4;
    while (*args == ' ') args++;
    
    // Find the delimiter (either '=' or space)
    char* delim = strchr(args, '=');
    if (!delim) {
      delim = strchr(args, ' ');
    }
    
    if (!delim) {
      Serial.println("Error: Invalid SET syntax. Use: set <param> = <value>");
      return;
    }
    
    // Split the name and value
    *delim = '\0';
    char* param_name = args;
    char* val_str = delim + 1;
    
    // Clean up param_name (trim trailing spaces)
    int name_len = strlen(param_name);
    while (name_len > 0 && (param_name[name_len - 1] == ' ' || param_name[name_len - 1] == '\t')) {
      param_name[name_len - 1] = '\0';
      name_len--;
    }
    
    // Clean up val_str (trim leading spaces)
    while (*val_str == ' ' || *val_str == '\t' || *val_str == '=') val_str++;
    
    if (strlen(param_name) == 0 || strlen(val_str) == 0) {
      Serial.println("Error: Missing parameter name or value.");
      return;
    }
    
    double val = atof(val_str);
    
    if (strncasecmp(param_name, "pin_", 4) == 0) {
      char* pin_param_name = param_name + 4;
      if (pin_config_set(pin_param_name, (int)val)) {
        Serial.print("Set pin_");
        Serial.print(pin_param_name);
        Serial.print(" = ");
        Serial.println((int)val);
      } else {
        Serial.println("Error: Pin parameter not found.");
      }
    } else {
      if (gnc_config_set(param_name, val)) {
        Serial.print("Set ");
        Serial.print(param_name);
        Serial.print(" = ");
        Serial.println(val, 6);
      } else {
        Serial.println("Error: GNC parameter not found.");
      }
    }
    return;
  }
  
  Serial.print("Error: Unknown command: ");
  Serial.println(line);
}

void cli_handler_update() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      cli_buffer[cli_index] = '\0';
      if (cli_index > 0) {
        parse_and_execute(cli_buffer);
        cli_index = 0;
      }
    } else if (cli_index < BUFFER_SIZE - 1) {
      cli_buffer[cli_index++] = c;
    }
  }
}
