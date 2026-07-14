#pragma once
#include <cstdint>
#include <SPI.h>

enum class LSM6DSV16X_ODR {
    OFF = 0,
    ODR_1Hz875 = 1,
    ODR_7Hz5 = 2,
    ODR_15Hz = 3,
    ODR_30Hz = 4,
    ODR_60Hz = 5,
    ODR_120Hz = 6,
    ODR_240Hz = 7,
    ODR_480Hz = 8,
    ODR_960Hz = 9,
    ODR_1920Hz = 10,
    ODR_3840Hz = 11,
    ODR_7680Hz = 12
};

enum class LSM6DSV16X_ACC_FS {
    FS_2G,
    FS_4G,
    FS_8G,
    FS_16G
};

enum class LSM6DSV16X_GYRO_FS {
    FS_125DPS,
    FS_250DPS,
    FS_500DPS,
    FS_1000DPS,
    FS_2000DPS,
    FS_4000DPS
};

struct MOTc {
    uint8_t m1_pin;
    uint8_t m2_pin;
    uint8_t m3_pin;
    uint8_t m4_pin;
    uint32_t speed_kbd;
    uint8_t pole_pairs;
};

struct RCRXc {
    uint8_t uart_id;
    uint8_t roll_ch;
    uint8_t pitch_ch;
    uint8_t thr_ch;
    uint8_t yaw_ch;
    uint8_t arm_ch;
    uint8_t mode_ch;
    float telemetry_hz;
};

struct BATc {
    uint8_t pin;
    float division_factor;
};

struct IMUc {
    SPIClass* spi_port;
    uint8_t cs_pin;
    LSM6DSV16X_ACC_FS accel_fs;
    LSM6DSV16X_GYRO_FS gyro_fs;
    LSM6DSV16X_ODR accel_odr;
    LSM6DSV16X_ODR gyro_odr;
};

struct SERc {
    uint8_t s1_pin;
    uint8_t s2_pin;
    uint8_t s3_pin;
    uint8_t s4_pin;
    uint16_t min_us;
    uint16_t max_us;
};

struct LEDc {
    uint8_t pin;
};

struct RPIc {
    // dummy config
};

struct HALc {
    MOTc motc;
    RCRXc rcrxc;
    BATc batc;
    IMUc imuc;
    SERc serc;
    LEDc ledc;
    RPIc rpic;
    uint8_t telemetry_uart_id;
    uint32_t telemetry_decimation;
};
