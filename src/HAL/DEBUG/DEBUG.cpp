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

    Serial.printf("$DBG,%.4f,%.4f\n",
                  allb_km1.halb.execution_time_ms,
                  allb_km1.gnc_time_ms);
}
