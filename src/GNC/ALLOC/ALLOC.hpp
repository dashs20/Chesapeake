#pragma once
#include "../bus.hpp"
#include "../cfg.hpp"

class ALLOC {
public:
    ALLOC(GNCc cfg);
    ACTb update(const ALLb& allb);

private:
    GNCc cfg_data;

    ACTb run_allocator(const ALLb& allb);
    ACTb clamp_actuators(const ALLb& allb, ACTb actb, float min_motor_frac);
    ACTb allocate_quad(const ALLb& allb);
    float determine_blink_hz(bool armed, FLIGHT_MODE mode);
};
