#include "GNC.hpp"

GNC::GNC(GNCc cfg) 
    : cfg_data(cfg),
      gnc_bus{},
      nav(cfg),
      vsm(cfg),
      gui(cfg),
      ctl(cfg),
      alloc(cfg) {}

ACTb GNC::update(const HALb& halb) {
    gnc_bus.halb = halb;
    gnc_bus.vsmb = vsm.update(gnc_bus);
    gnc_bus.navb = nav.update(gnc_bus);
    gnc_bus.guib = gui.update(gnc_bus);
    gnc_bus.ctlb = ctl.update(gnc_bus);
    gnc_bus.actb = alloc.update(gnc_bus);

    return gnc_bus.actb;
}
