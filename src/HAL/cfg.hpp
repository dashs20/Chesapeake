#pragma once
#include <cstdint>

struct MOTc {
    uint8_t start_pin;
    uint8_t num_pins;
    uint32_t speed_kbd;
};

struct RCRXc {
    uint8_t uart_id;
    uint8_t roll_ch;
    uint8_t pitch_ch;
    uint8_t thr_ch;
    uint8_t yaw_ch;
    uint8_t arm_ch;
    uint8_t mode_ch;
};

struct BATc {
    uint8_t pin;
    float division_factor;
};

struct HALc {
    MOTc motc;
    RCRXc rcrxc;
    BATc batc;
};
