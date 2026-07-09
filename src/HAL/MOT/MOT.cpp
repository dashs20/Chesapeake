#include "MOT.hpp"
#include <algorithm>

MOT::MOT(MOTc cfg) : motc(cfg) {
    for (uint8_t i = 0; i < 4; ++i) {
        motors[i] = nullptr;
    }
    if (motc.m1_pin != 255) motors[0] = new BidirDShotX1(motc.m1_pin, motc.speed_kbd);
    if (motc.m2_pin != 255) motors[1] = new BidirDShotX1(motc.m2_pin, motc.speed_kbd);
    if (motc.m3_pin != 255) motors[2] = new BidirDShotX1(motc.m3_pin, motc.speed_kbd);
    if (motc.m4_pin != 255) motors[3] = new BidirDShotX1(motc.m4_pin, motc.speed_kbd);
}

MOT::~MOT() {
    for (uint8_t i = 0; i < 4; ++i) {
        delete motors[i];
    }
}

MOTb MOT::update(const ACTb& actb) {
    if (motors[0] != nullptr && !motors[0]->initError()) {
        float m1_frac = std::clamp(actb.m1_frac, 0.0f, 1.0f);
        motors[0]->sendThrottle(static_cast<uint16_t>(m1_frac * 2000.0f));
    }
    if (motors[1] != nullptr && !motors[1]->initError()) {
        float m2_frac = std::clamp(actb.m2_frac, 0.0f, 1.0f);
        motors[1]->sendThrottle(static_cast<uint16_t>(m2_frac * 2000.0f));
    }
    if (motors[2] != nullptr && !motors[2]->initError()) {
        float m3_frac = std::clamp(actb.m3_frac, 0.0f, 1.0f);
        motors[2]->sendThrottle(static_cast<uint16_t>(m3_frac * 2000.0f));
    }
    if (motors[3] != nullptr && !motors[3]->initError()) {
        float m4_frac = std::clamp(actb.m4_frac, 0.0f, 1.0f);
        motors[3]->sendThrottle(static_cast<uint16_t>(m4_frac * 2000.0f));
    }

    MOTb motb;
    float divisor = (motc.pole_pairs > 0) ? static_cast<float>(motc.pole_pairs) : 1.0f;

    uint32_t erpm_m1 = 0;
    if (motors[0] != nullptr && motors[0]->getTelemetryErpm(&erpm_m1) == BidirDshotTelemetryType::ERPM) {
        motb.m1_rpm = static_cast<float>(erpm_m1) / divisor;
    } else {
        motb.m1_rpm = 0.0f;
    }

    uint32_t erpm_m2 = 0;
    if (motors[1] != nullptr && motors[1]->getTelemetryErpm(&erpm_m2) == BidirDshotTelemetryType::ERPM) {
        motb.m2_rpm = static_cast<float>(erpm_m2) / divisor;
    } else {
        motb.m2_rpm = 0.0f;
    }

    uint32_t erpm_m3 = 0;
    if (motors[2] != nullptr && motors[2]->getTelemetryErpm(&erpm_m3) == BidirDshotTelemetryType::ERPM) {
        motb.m3_rpm = static_cast<float>(erpm_m3) / divisor;
    } else {
        motb.m3_rpm = 0.0f;
    }

    uint32_t erpm_m4 = 0;
    if (motors[3] != nullptr && motors[3]->getTelemetryErpm(&erpm_m4) == BidirDshotTelemetryType::ERPM) {
        motb.m4_rpm = static_cast<float>(erpm_m4) / divisor;
    } else {
        motb.m4_rpm = 0.0f;
    }

    return motb;
}
