#include "DEBUG.hpp"

DEBUG::DEBUG(DEBUGc cfg) : debugc(cfg), counter(0) {}

DEBUG::~DEBUG() {}

void DEBUG::update(const ALLb& allb_km1) {
    if (!debugc.enabled) {
        return;
    }

    counter++;
    if (debugc.decimation > 0 && (counter % debugc.decimation) != 0) {
        return;
    }

    const char* state_str = "UNKNOWN";
    if (allb_km1.gncb.vsmb.state == STATE::DISARMED) {
        state_str = "DISARMED";
    } else if (allb_km1.gncb.vsmb.state == STATE::RATE) {
        state_str = "RATE";
    } else if (allb_km1.gncb.vsmb.state == STATE::ANGLE) {
        state_str = "ANGLE";
    }

    float m1 = allb_km1.gncb.actb.m1_frac;
    float m2 = allb_km1.gncb.actb.m2_frac;
    float m3 = allb_km1.gncb.actb.m3_frac;
    float m4 = allb_km1.gncb.actb.m4_frac;

    Serial.printf("$DBG,STATE: %s | M1: %0.4f | M2: %0.4f | M3: %0.4f | M4: %0.4f\n",
                  state_str, m1, m2, m3, m4);
}
