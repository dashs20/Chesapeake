#include "HAL.hpp"

HAL::HAL(HALc cfg) 
    : cfg_data(cfg),
      hal_bus{},
      imu(cfg.imuc),
      rcrx(cfg.rcrxc),
      mot(cfg.motc),
      bat(cfg.batc),
      ser(cfg.serc),
      led(cfg.ledc.pin),
      rpi(cfg.rpic),
      debug(cfg.debugc) {}

HAL::~HAL() {}

HALb HAL::update(const ALLb& allb_km1) {
    hal_bus.imub = imu.update(hal_bus);
    hal_bus.rcrxb = rcrx.update(hal_bus);
    mot.update(allb_km1.gncb.actb);
    hal_bus.vbat_volts = bat.update(hal_bus);
    ser.update(allb_km1.gncb.actb);
    led.update(allb_km1.gncb.actb);
    rpi.update(allb_km1);
    debug.update(allb_km1);

    return hal_bus;
}
