#include "HAL.hpp"

HAL::HAL(HALc cfg, uint32_t looprate_hz) 
    : cfg_data(cfg),
      hal_bus{},
      imu(cfg.imuc),
      rcrx(cfg.rcrxc, looprate_hz),
      mot(cfg.motc),
      bat(cfg.batc),
      ser(cfg.serc),
      led(cfg.ledc.pin),
      rpi(cfg.rpic) {}

HAL::~HAL() {}

HALb HAL::update(const ALLb& allb_km1) {
    mot.update(allb_km1.gncb.actb);
    ser.update(allb_km1.gncb.actb);
    led.update(allb_km1.gncb.actb);
    rpi.update(allb_km1);

    PKG::send_packet(allb_km1, &Serial);

    hal_bus.imub = imu.update(hal_bus);
    hal_bus.rcrxb = rcrx.update(hal_bus);
    hal_bus.vbat_volts = bat.update(hal_bus);

    return hal_bus;
}
