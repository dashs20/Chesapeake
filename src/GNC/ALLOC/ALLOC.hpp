#pragma once
#include "../bus.hpp"
#include "../cfg.hpp"

class ALLOC {
public:
    ALLOC(GNCc cfg);
    ACTb update(const GNCb& gnc);

private:
    GNCc cfg_data;

    ACTb run_allocator(const GNCb& gnc);
    ACTb clamp_actuators(const GNCb& gnc, ACTb actb);
    ACTb allocate_quad(const GNCb& gnc);
};
