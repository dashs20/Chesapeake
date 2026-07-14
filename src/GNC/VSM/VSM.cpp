#include "VSM.hpp"

VSM::VSM(GNCc cfg) : cfg_data(cfg) {}

VSM::~VSM() {}

VSMb VSM::update(const ALLb& allb) {
    VSMb vsmb;
    vsmb.armed = (allb.halb.rcrxb.arm_frac > cfg_data.vsmc.arm_threshold_frac);

    if (allb.cfg_appb.act_test.enabled) {
        vsmb.mode = FLIGHT_MODE::ACT_TEST;
        vsmb.armed = true;
    } else if (allb.halb.rcrxb.mode_frac < cfg_data.vsmc.mode_rate_threshold_frac) {
        vsmb.mode = FLIGHT_MODE::RATE;
    } else {
        vsmb.mode = FLIGHT_MODE::ANGLE;
    }

    return vsmb;
}
