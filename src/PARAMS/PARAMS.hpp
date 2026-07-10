#pragma once

#include "MASTERc.hpp"
#include <EEPROM.h>

class PARAMS {
public:
    PARAMS();
    ~PARAMS();

    bool load(MASTERc& config);
    void save(const MASTERc& config);
    void run_cli(MASTERc& config);

private:
    void print_help();
    void print_all(const MASTERc& config);
    void get_parameter(const MASTERc& config, const char* name);
    void set_parameter(MASTERc& config, const char* name, const char* value);

    bool is_calibrating;
    int calibration_counter;
    float sum_ax;
    float sum_ay;
    float sum_az;
    float sum_gx;
    float sum_gy;
    float sum_gz;
};
