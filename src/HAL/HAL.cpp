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
      rpi(cfg.rpic),
      telemetry_counter(0) {}

HAL::~HAL() {}

HALb HAL::update(const ALLb& allb_km1) {
    hal_bus.imub = imu.update(hal_bus);
    hal_bus.rcrxb = rcrx.update(hal_bus);
    hal_bus.vbat_volts = bat.update(hal_bus);

    mot.update(allb_km1.gncb.actb);
    ser.update(allb_km1.gncb.actb);
    led.update(allb_km1.gncb.actb);
    rpi.update(allb_km1);

    telemetry_counter++;
    if (cfg_data.telemetry_decimation == 0 || (telemetry_counter % cfg_data.telemetry_decimation) == 0) {
        Stream* telem_port = &Serial;
        if (cfg_data.telemetry_uart_id == 1) {
            telem_port = &Serial1;
        } else if (cfg_data.telemetry_uart_id == 2) {
            telem_port = &Serial2;
        }
        PKG::send_packet(allb_km1, telem_port);
    }

    return hal_bus;
}
