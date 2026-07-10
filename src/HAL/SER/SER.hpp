#pragma once

#include "../cfg.hpp"
#include "../../GNC/bus.hpp"
#include <Servo.h>

class SER {
public:
    SER(SERc cfg);
    ~SER();

    void update(const ACTb& actb);

private:
    SERc serc;
    Servo servos[4];
};
