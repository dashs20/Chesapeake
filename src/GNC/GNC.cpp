#include "GNC.hpp"

GNC::GNC(GNCc cfg) 
    : cfg_data(cfg),
      nav(cfg),
      vsm(cfg),
      gui(cfg),
      ctl(cfg),
      alloc(cfg) {}

GNCb GNC::update(GNCb gnc) {
    gnc.vsmb = vsm.update(gnc);
    gnc.navb = nav.update(gnc);
    gnc.guib = gui.update(gnc);
    gnc.ctlb = ctl.update(gnc);
    gnc.actb = alloc.update(gnc);

    return gnc;
}
