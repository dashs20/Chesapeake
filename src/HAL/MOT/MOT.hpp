#pragma once

#include "../cfg.hpp"
#include "../../GNC/bus.hpp"
#include <PIO_DShot.h>

class MOT {
public:
    MOT(MOTc cfg);
    ~MOT();

    MOTb update(const ACTb& actb);

private:
    MOTc motc;
    BidirDShotX1* motors[4];
};
