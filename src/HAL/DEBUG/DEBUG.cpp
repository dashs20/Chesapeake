#include "DEBUG.hpp"

DEBUG::DEBUG(DEBUGc cfg) : debugc(cfg) {}

DEBUG::~DEBUG() {}

void DEBUG::update(const ALLb& allb_km1) {
    if (!debugc.enabled) {
        return;
    }

    Serial.printf("$DBG,%.4f,%.4f\n",
                  allb_km1.navb.euler_bodyz2up_rad.x() * 57.2958f,
                  allb_km1.navb.euler_bodyz2up_rad.y() * 57.2958f);
}
