#pragma once

#include "bus.hpp"
#include "cfg.hpp"
#include "IMU/IMU.hpp"
#include "RCRX/RCRX.hpp"
#include "MOT/MOT.hpp"
#include "BAT/BAT.hpp"
#include "SERVO/SERVO.hpp"
#include "LED/LED.hpp"

class HAL {
public:
    HAL(HALc cfg);
    ~HAL();

    HALb update(const ACTb& actb);

private:
    HALc cfg_data;
    HALb hal_bus;
    IMU imu;
    RCRX rcrx;
    MOT mot;
    BAT bat;
    SERVO servo;
    LED led;
};
