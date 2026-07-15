#pragma once

#include "../cfg.hpp"
#include "../bus.hpp"
#include <AlfredoCRSF.h>

class RCRX {
public:
    RCRX(RCRXc rcrxc, uint32_t looprate_hz);
    ~RCRX();

    RCRXb update(const HALb& halb);

private:
    RCRXc rcrxc;
    Stream* port = nullptr;
    AlfredoCRSF crsf;
    float dt_s;
    float telemetry_dt_s;
    float time_accumulator_s;
    bool is_initialized = false;
};
