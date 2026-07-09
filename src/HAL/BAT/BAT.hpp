#pragma once

#include "../cfg.hpp"
#include "../bus.hpp"

class BAT {
public:
    BAT(BATc cfg);
    ~BAT();

    float update(const HALb& halb);

private:
    BATc batc;
};
