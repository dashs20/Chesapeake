#pragma once
#include "../bus.hpp"
#include "../cfg.hpp"

class ALLOC {
public:
    ALLOC(GNCc cfg);
    GNCb update(GNCb gnc);

private:
    GNCc cfg_data;

    GNCb run_allocator(GNCb gnc);
    GNCb clamp_actuators(GNCb gnc);
    GNCb allocate_quad(GNCb gnc);
};
