#pragma once
#include "../bus.hpp"
#include "../cfg.hpp"

class GUI {
public:
    GUI(GNCc cfg);
    GNCb update(GNCb gnc);

private:
    GNCc cfg_data;
    float apply_expo(float input, float expo);
};
