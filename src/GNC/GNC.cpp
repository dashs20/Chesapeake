#include "GNC.hpp"

GNC::GNC(GNCc cfg) 
    : cfg_data(cfg),
      dt_s(1.0f / static_cast<float>(cfg.looprate_hz)),
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

void GNC::update_dual_core(GNCb& gnc_k, const GNCb& gnc_km1) {
    gnc_k.vsmb = vsm.update(gnc_km1);
    gnc_k.navb = nav.update(gnc_km1);
    gnc_k.guib = gui.update(gnc_km1);
    gnc_k.ctlb = ctl.update(gnc_km1);
    gnc_k.actb = alloc.update(gnc_km1);
}
