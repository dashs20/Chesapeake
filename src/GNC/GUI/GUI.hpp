#pragma once
#include "../bus.hpp"
#include "../cfg.hpp"

class GUI {
public:
    GUI(GNCc cfg);
    GUIb update(const ALLb& allb);

private:
    GNCc cfg_data;
    float apply_expo(float input_frac, float expo_frac);
};
