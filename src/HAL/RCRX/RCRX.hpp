#pragma once

#include "../cfg.hpp"
#include "../bus.hpp"
#include <AlfredoCRSF.h>

class RCRX {
public:
    RCRX(RCRXc rcrxc);
    ~RCRX();

    RCRXb update(const HALb& halb);

private:
    RCRXc rcrxc;
    Stream* port = nullptr;
    AlfredoCRSF crsf;
    uint32_t last_telemetry_ms = 0;
};
