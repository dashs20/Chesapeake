#pragma once
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
    void update_dual_core(GNCb& gnc_k, const GNCb& gnc_km1);
    const GNCb& get_bus() const { return gnc_bus; }
    void reset_estimator() { nav.reset(); }

private:
    GNCc cfg_data;
    float dt_s;
    GNCb gnc_bus;
    NAV nav;
    VSM vsm;
    GUI gui;
    CTL ctl;
    ALLOC alloc;
};
