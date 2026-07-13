#pragma once

#include "../cfg.hpp"
#include "../../GNC/bus.hpp"
#include <PIO_DShot.h>

class MOT {
public:
    MOT(MOTc cfg);
    ~MOT();

    void update(const ACTb& actb);

private:
    MOTc motc;
    DShotX4* dshot1;
    DShotX4* dshot2;
};
