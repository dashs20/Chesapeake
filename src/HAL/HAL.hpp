#pragma once

#include "bus.hpp"
#include "cfg.hpp"
#include "IMU/IMU.hpp"
#include "RCRX/RCRX.hpp"
#include "MOT/MOT.hpp"
#include "BAT/BAT.hpp"
#include "SER/SER.hpp"
#include "LED/LED.hpp"
#include "RPI/RPI.hpp"
#include "PKG/PKG.hpp"

class HAL {
public:
    HAL(HALc cfg, uint32_t looprate_hz);
    ~HAL();

    HALb update(const ALLb& allb_km1);
    const HALb& get_bus() const { return hal_bus; }

private:
    HALc cfg_data;
    HALb hal_bus;
    IMU imu;
    RCRX rcrx;
    MOT mot;
    BAT bat;
    SER ser;
    LED led;
    RPI rpi;
    uint32_t telemetry_counter;
};
