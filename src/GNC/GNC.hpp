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

private:
    GNCc cfg_data;
    GNCb gnc_bus;
    NAV nav;
    VSM vsm;
    GUI gui;
    CTL ctl;
    ALLOC alloc;
};
