#pragma once

#include "../cfg.hpp"
#include "../../GNC/bus.hpp"
#include <Arduino.h>

class RPI {
public:
    RPI(RPIc cfg);
    ~RPI();

    void update(const ALLb& allb_km1);

private:
    RPIc rpic;
    Stream* port;
    uint32_t loop_count;
};
