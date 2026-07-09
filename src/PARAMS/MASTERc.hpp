#pragma once

#include "../HAL/cfg.hpp"
#include "../GNC/cfg.hpp"

struct MASTERc {
    uint32_t magic;
    HALc halc;
    GNCc gncc;
};
