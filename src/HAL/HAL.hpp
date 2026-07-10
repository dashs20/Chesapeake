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
#include "DEBUG/DEBUG.hpp"

class HAL {
public:
    HAL(HALc cfg);
    ~HAL();

    HALb update(const ALLb& allb_km1);

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
    DEBUG debug;
};
