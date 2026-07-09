#include "HAL.hpp"

HAL::HAL(HALc cfg) 
    : cfg_data(cfg),
      hal_bus{},
      imu(cfg.imuc),
      rcrx(cfg.rcrxc),
      mot(cfg.motc),
      bat(cfg.batc),
      ser(cfg.serc),
      led(cfg.ledc.pin) {}

HAL::~HAL() {}

HALb HAL::update(const ACTb& actb) {
    hal_bus.imub = imu.update(hal_bus);
    hal_bus.rcrxb = rcrx.update(hal_bus);
    hal_bus.motb = mot.update(actb);
    hal_bus.vbat_volts = bat.update(hal_bus);
    ser.update(actb);
    led.update(actb);

    return hal_bus;
}
