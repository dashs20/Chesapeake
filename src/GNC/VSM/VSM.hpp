#pragma once

#include "../bus.hpp"
#include "../cfg.hpp"

class VSM {
public:
    VSM(GNCc cfg);
    ~VSM();
    VSMb update(const ALLb& allb);

private:
    GNCc cfg_data;
};
