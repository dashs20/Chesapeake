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
    GNCb update(GNCb gnc);

private:
    GNCc cfg_data;
    NAV nav;
    VSM vsm;
    GUI gui;
    CTL ctl;
    ALLOC alloc;
};
