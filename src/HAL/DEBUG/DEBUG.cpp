#include "DEBUG.hpp"

DEBUG::DEBUG(DEBUGc cfg) : debugc(cfg) {}

DEBUG::~DEBUG() {}

void DEBUG::update(const ALLb& allb_km1) {
    if (!debugc.enabled) {
        return;
    }

    Serial.printf("$DBG,%.4f,%.4f,%.4f\n", 
                  allb_km1.navb.omega_body_radps.x(),
                  allb_km1.navb.omega_body_radps.y(),
                  allb_km1.navb.omega_body_radps.z());
}
