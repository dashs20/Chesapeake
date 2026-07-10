#include "DEBUG.hpp"

DEBUG::DEBUG(DEBUGc cfg) : debugc(cfg) {}

DEBUG::~DEBUG() {}

void DEBUG::update(const ALLb& allb_km1) {
    if (!debugc.enabled) {
        return;
    }

    Serial.printf("$DBG,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
                  allb_km1.halb.imub.accel_body_mps2.x(),
                  allb_km1.halb.imub.accel_body_mps2.y(),
                  allb_km1.halb.imub.accel_body_mps2.z(),
                  allb_km1.halb.imub.omega_body_radps.x(),
                  allb_km1.halb.imub.omega_body_radps.y(),
                  allb_km1.halb.imub.omega_body_radps.z(),
                  allb_km1.navb.euler_bodyz2up_rad.x() * 57.2958f,
                  allb_km1.navb.euler_bodyz2up_rad.y() * 57.2958f);
}
