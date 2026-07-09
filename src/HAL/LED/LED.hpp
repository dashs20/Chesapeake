#pragma once

#include "../cfg.hpp"
#include "../../GNC/bus.hpp"
#include <cstdint>

class LED {
public:
    LED(uint8_t pin);
    ~LED();

    void update(const ACTb& actb);

private:
    uint8_t pin_num;
};
