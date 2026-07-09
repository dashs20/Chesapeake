#pragma once

#include "../cfg.hpp"
#include "../bus.hpp"
#include <LSM6DSV16XSensor.h>

class IMU {
public:
    IMU(IMUc cfg);
    ~IMU();

    IMUb update(const HALb& halb);

private:
    IMUc imuc;
    LSM6DSV16XSensor* sensor;

    float convert_odr_to_float(LSM6DSV16X_ODR odr);
    int32_t convert_accel_fs_to_int(LSM6DSV16X_ACC_FS fs);
    int32_t convert_gyro_fs_to_int(LSM6DSV16X_GYRO_FS fs);
};
