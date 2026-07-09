#pragma once

#include "../cfg.hpp"
#include "../../GNC/bus.hpp"
#include <Arduino.h>

class DEBUG {
public:
    DEBUG(DEBUGc cfg);
    ~DEBUG();

    void update(const ALLb& allb_km1);

private:
    DEBUGc debugc;
};
