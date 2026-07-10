#pragma once
#include <Arduino.h>
#include "bus.hpp"
#include "cfg.hpp"
#include "NAV/NAV.hpp"
#include "VSM/VSM.hpp"
#include "GUI/GUI.hpp"
#include "CTL/CTL.hpp"
#include "ALLOC/ALLOC.hpp"

class GNC {
public:
    GNC(GNCc cfg);
    ACTb update(const HALb& halb);
    void update_dual_core(ALLb& allb_k, const ALLb& allb_km1);
    void reset_estimator() { nav.reset(); }

private:
    GNCb update_dual_core_internal(const ALLb& allb_km1);

    GNCc cfg_data;
    float dt_s;
    NAV nav;
    VSM vsm;
    GUI gui;
    CTL ctl;
    ALLOC alloc;
};
