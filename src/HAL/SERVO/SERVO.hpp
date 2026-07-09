#pragma once

#include "../cfg.hpp"
#include "../../GNC/bus.hpp"
#include <Servo.h>

class SERVO {
public:
    SERVO(SERVOc cfg);
    ~SERVO();

    void update(const ACTb& actb);

private:
    SERVOc servoc;
    Servo servos[4];
};
