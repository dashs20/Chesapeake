#include "MOT.hpp"
#include <algorithm>

MOT::MOT(MOTc motc) : motc(motc) {
    dshot = new DShotX4(motc.start_pin, motc.num_pins, motc.speed_kbd);
}

MOT::~MOT() {
    delete dshot;
}

void MOT::update(const ACTb& actb) {
    if (dshot == nullptr || dshot->initError()) {
        return;
    }

    uint16_t throttles[4] = {0, 0, 0, 0};

    if (motc.num_pins > 0) {
        float m1_frac = std::clamp(actb.m1_frac, 0.0f, 1.0f);
        throttles[0] = static_cast<uint16_t>(m1_frac * 2000.0f);
    }
    if (motc.num_pins > 1) {
        float m2_frac = std::clamp(actb.m2_frac, 0.0f, 1.0f);
        throttles[1] = static_cast<uint16_t>(m2_frac * 2000.0f);
    }
    if (motc.num_pins > 2) {
        float m3_frac = std::clamp(actb.m3_frac, 0.0f, 1.0f);
        throttles[2] = static_cast<uint16_t>(m3_frac * 2000.0f);
    }
    if (motc.num_pins > 3) {
        float m4_frac = std::clamp(actb.m4_frac, 0.0f, 1.0f);
        throttles[3] = static_cast<uint16_t>(m4_frac * 2000.0f);
    }

    dshot->sendThrottles(throttles);
}
