#pragma once

#include <Arduino.h>
#include "../GNC/bus.hpp"

class Packager {
public:
    static uint16_t calculate_fletcher16(const uint8_t* data, size_t length);
    static size_t package_allb(const ALLb& allb, uint8_t* buffer, size_t max_len);
};
