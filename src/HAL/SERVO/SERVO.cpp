#include "SERVO.hpp"
#include <algorithm>

SERVO::SERVO(SERVOc cfg) : servoc(cfg) {
    if (servoc.s1_pin != 255) {
        servos[0].attach(servoc.s1_pin, static_cast<int>(servoc.min_us), static_cast<int>(servoc.max_us));
    }
    if (servoc.s2_pin != 255) {
        servos[1].attach(servoc.s2_pin, static_cast<int>(servoc.min_us), static_cast<int>(servoc.max_us));
    }
    if (servoc.s3_pin != 255) {
        servos[2].attach(servoc.s3_pin, static_cast<int>(servoc.min_us), static_cast<int>(servoc.max_us));
    }
    if (servoc.s4_pin != 255) {
        servos[3].attach(servoc.s4_pin, static_cast<int>(servoc.min_us), static_cast<int>(servoc.max_us));
    }
}

SERVO::~SERVO() {
    for (uint8_t i = 0; i < 4; ++i) {
        servos[i].detach();
    }
}

void SERVO::update(const ACTb& actb) {
    if (servoc.s1_pin != 255) {
        servos[0].write(static_cast<int>(std::clamp(actb.s1_deg, 0.0f, 180.0f)));
    }
    if (servoc.s2_pin != 255) {
        servos[1].write(static_cast<int>(std::clamp(actb.s2_deg, 0.0f, 180.0f)));
    }
    if (servoc.s3_pin != 255) {
        servos[2].write(static_cast<int>(std::clamp(actb.s3_deg, 0.0f, 180.0f)));
    }
    if (servoc.s4_pin != 255) {
        servos[3].write(static_cast<int>(std::clamp(actb.s4_deg, 0.0f, 180.0f)));
    }
}
