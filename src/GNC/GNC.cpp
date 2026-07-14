#include "GNC.hpp"

GNC::GNC(GNCc cfg) 
    : cfg_data(cfg),
      dt_s(1.0f / static_cast<float>(cfg.looprate_hz)),
      nav(cfg),
      vsm(cfg),
      gui(cfg),
      ctl(cfg),
      alloc(cfg) {}

ACTb GNC::update(const HALb& halb) {
    ALLb temp_allb;
    temp_allb.halb = halb;
    temp_allb.gncb = update_dual_core_internal(temp_allb);
    return temp_allb.gncb.actb;
}

void GNC::update_dual_core(ALLb& allb_k, const ALLb& allb_km1) {
    uint32_t gnc_start = micros();
    allb_k.gncb = update_dual_core_internal(allb_km1);
    allb_k.gncb.gnc_time_ms = (micros() - gnc_start) / 1000.0f;
}

GNCb GNC::update_dual_core_internal(const ALLb& allb_km1) {
    GNCb gncb;
    gncb.vsmb = vsm.update(allb_km1);

    uint32_t t_start = micros();
    gncb.navb = nav.update(allb_km1);
    gncb.time_nav_us = static_cast<float>(micros() - t_start);

    gncb.guib = gui.update(allb_km1);

    t_start = micros();
    gncb.ctlb = ctl.update(allb_km1);
    gncb.time_ctl_us = static_cast<float>(micros() - t_start);

    t_start = micros();
    gncb.actb = alloc.update(allb_km1);
    gncb.time_alloc_us = static_cast<float>(micros() - t_start);

    return gncb;
}
