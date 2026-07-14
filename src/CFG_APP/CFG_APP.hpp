#pragma once

#include "../GNC/bus.hpp"
#include "../PARAMS/PARAMS.hpp"

class CFG_APP {
public:
    CFG_APP(const MASTERc& config);
    ~CFG_APP();

    CFG_APPb update(const ALLb& allb, MASTERc& config, PARAMS& params);

    void clear_save_request() { cfg_appb.save_requested = false; }
    void clear_reboot_request() { cfg_appb.reboot_requested = false; }
    void clear_defaults_request() { cfg_appb.defaults_requested = false; }
    void clear_calibrate_request() { cfg_appb.calibrate_requested = false; }

private:
    CFG_APPb cfg_appb;

    void print_help();
};
