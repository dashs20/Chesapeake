#pragma once
#include "../HAL/bus.hpp"

struct ACT_TESTb {
    bool enabled;
    ACTb commands;
};

struct CFG_APPb {
    bool calibrate_requested;
    bool reboot_requested;
    bool save_requested;
    bool defaults_requested;
    bool is_calibrating;
    float calibration_progress_frac;
    ACT_TESTb act_test;
};
