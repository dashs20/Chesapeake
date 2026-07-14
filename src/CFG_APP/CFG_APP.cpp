#include "CFG_APP.hpp"
#include <Arduino.h>
#include <cstring>
#include <cstdlib>

CFG_APP::CFG_APP(const MASTERc& config) {
    cfg_appb.calibrate_requested = false;
    cfg_appb.reboot_requested = false;
    cfg_appb.save_requested = false;
    cfg_appb.defaults_requested = false;
    cfg_appb.is_calibrating = false;
    cfg_appb.calibration_progress_frac = 0.0f;
    cfg_appb.act_test.enabled = false;
    cfg_appb.act_test.commands.m1_frac = 0.0f;
    cfg_appb.act_test.commands.m2_frac = 0.0f;
    cfg_appb.act_test.commands.m3_frac = 0.0f;
    cfg_appb.act_test.commands.m4_frac = 0.0f;
    cfg_appb.act_test.commands.s1_deg = config.gncc.allocc.ser_default_ang_deg;
    cfg_appb.act_test.commands.s2_deg = config.gncc.allocc.ser_default_ang_deg;
    cfg_appb.act_test.commands.s3_deg = config.gncc.allocc.ser_default_ang_deg;
    cfg_appb.act_test.commands.s4_deg = config.gncc.allocc.ser_default_ang_deg;
    cfg_appb.act_test.commands.LED_blink_Hz = 0.0f;
}

CFG_APP::~CFG_APP() {}

CFG_APPb CFG_APP::update(const ALLb& allb, MASTERc& config, PARAMS& params) {
    cfg_appb.is_calibrating = false;
    cfg_appb.calibration_progress_frac = 0.0f;

    static char buf[128];
    static size_t idx = 0;

    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\r' || c == '\n') {
            if (idx > 0) {
                buf[idx] = '\0';
                
                char* cmd = std::strtok(buf, " ");
                if (cmd != nullptr) {
                    if (std::strcmp(cmd, "help") == 0) {
                        print_help();
                    } else if (std::strcmp(cmd, "dump") == 0) {
                        params.print_all(config);
                    } else if (std::strcmp(cmd, "defaults") == 0) {
                        cfg_appb.defaults_requested = true;
                        Serial.println("Load defaults requested...");
                    } else if (std::strcmp(cmd, "reboot") == 0) {
                        cfg_appb.reboot_requested = true;
                        Serial.println("Reboot requested...");
                    } else if (std::strcmp(cmd, "save") == 0) {
                        cfg_appb.save_requested = true;
                        Serial.println("Save config requested...");
                    } else if (std::strcmp(cmd, "calibrate") == 0) {
                        cfg_appb.calibrate_requested = true;
                        Serial.println("Calibration requested... Keep the board flat and still.");
                    } else if (std::strcmp(cmd, "act_test") == 0) {
                        char* enabled_str = std::strtok(nullptr, " ");
                        char* m1_str = std::strtok(nullptr, " ");
                        char* m2_str = std::strtok(nullptr, " ");
                        char* m3_str = std::strtok(nullptr, " ");
                        char* m4_str = std::strtok(nullptr, " ");
                        char* s1_str = std::strtok(nullptr, " ");
                        char* s2_str = std::strtok(nullptr, " ");
                        char* s3_str = std::strtok(nullptr, " ");
                        char* s4_str = std::strtok(nullptr, " ");
                        char* led_str = std::strtok(nullptr, " ");
                        if (enabled_str != nullptr && m1_str != nullptr && m2_str != nullptr && m3_str != nullptr && m4_str != nullptr &&
                            s1_str != nullptr && s2_str != nullptr && s3_str != nullptr && s4_str != nullptr && led_str != nullptr) {
                            cfg_appb.act_test.enabled = (std::atoi(enabled_str) != 0);
                            cfg_appb.act_test.commands.m1_frac = static_cast<float>(std::strtod(m1_str, nullptr));
                            cfg_appb.act_test.commands.m2_frac = static_cast<float>(std::strtod(m2_str, nullptr));
                            cfg_appb.act_test.commands.m3_frac = static_cast<float>(std::strtod(m3_str, nullptr));
                            cfg_appb.act_test.commands.m4_frac = static_cast<float>(std::strtod(m4_str, nullptr));
                            cfg_appb.act_test.commands.s1_deg = static_cast<float>(std::strtod(s1_str, nullptr));
                            cfg_appb.act_test.commands.s2_deg = static_cast<float>(std::strtod(s2_str, nullptr));
                            cfg_appb.act_test.commands.s3_deg = static_cast<float>(std::strtod(s3_str, nullptr));
                            cfg_appb.act_test.commands.s4_deg = static_cast<float>(std::strtod(s4_str, nullptr));
                            cfg_appb.act_test.commands.LED_blink_Hz = static_cast<float>(std::strtod(led_str, nullptr));
                        } else {
                            Serial.println("Error: Invalid act_test syntax.");
                        }
                    } else if (std::strcmp(cmd, "get") == 0) {
                        char* param_name = std::strtok(nullptr, " ");
                        if (param_name != nullptr) {
                            params.get_parameter(config, param_name);
                        } else {
                            Serial.println("Error: Missing parameter name");
                        }
                    } else if (std::strcmp(cmd, "set") == 0) {
                        char* param_name = std::strtok(nullptr, " =");
                        char* param_val = std::strtok(nullptr, " =");
                        if (param_name != nullptr && param_val != nullptr) {
                            params.set_parameter(config, param_name, param_val);
                        } else {
                            Serial.println("Error: Invalid set syntax. Use: set <param> = <value>");
                        }
                    } else {
                        Serial.println("Error: Unknown command. Type 'help' for options.");
                    }
                }
                idx = 0;
            }
        } else if (idx < sizeof(buf) - 1) {
            buf[idx++] = c;
        }
    }

    return cfg_appb;
}

void CFG_APP::print_help() {
    Serial.println("Chesapeake CLI Help (For Chesapeake supported boards, like Bluecrab)");
    Serial.println("Commands:");
    Serial.println("  help                       - Show this help list");
    Serial.println("  dump                       - Dump all parameter settings and current values");
    Serial.println("  get <param>                - Print the current value of a parameter");
    Serial.println("  set <param> = <value>      - Set a parameter in RAM (no spaces around '=')");
    Serial.println("  save                       - Save RAM configuration to flash and reboot");
    Serial.println("  defaults                   - Load default settings into RAM (requires save)");
    Serial.println("  calibrate                  - Calibrate IMU/UKF flat and save biases");
    Serial.println("  reboot                     - Reboot the flight controller");
}
