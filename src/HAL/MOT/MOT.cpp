#include "MOT.hpp"
#include <algorithm>
#include <Arduino.h>

MOT::MOT(MOTc cfg) : motc(cfg), dshot1(nullptr), dshot2(nullptr) {
    dshot1 = new DShotX4(motc.m1_pin, 2, motc.speed_kbd, pio0);
    if (dshot1->initError()) {
        delete dshot1;
        dshot1 = new DShotX4(motc.m1_pin, 2, motc.speed_kbd, pio1);
        if (dshot1->initError()) {
            Serial.printf("MOT: dshot1 FAILED on both pio0 and pio1!\n");
        } else {
            Serial.printf("MOT: dshot1 initialized on pio1\n");
        }
    } else {
        Serial.printf("MOT: dshot1 initialized on pio0\n");
    }

    dshot2 = new DShotX4(motc.m3_pin, 2, motc.speed_kbd, pio0);
    if (dshot2->initError()) {
        delete dshot2;
        dshot2 = new DShotX4(motc.m3_pin, 2, motc.speed_kbd, pio1);
        if (dshot2->initError()) {
            Serial.printf("MOT: dshot2 FAILED on both pio0 and pio1!\n");
        } else {
            Serial.printf("MOT: dshot2 initialized on pio1\n");
        }
    } else {
        Serial.printf("MOT: dshot2 initialized on pio0\n");
    }

    for (int count = 0; count < 1500; ++count) {
        uint16_t zero_throttles[4] = {0, 0, 0, 0};
        if (dshot1 != nullptr && !dshot1->initError()) {
            dshot1->sendThrottles(zero_throttles);
        }
        if (dshot2 != nullptr && !dshot2->initError()) {
            dshot2->sendThrottles(zero_throttles);
        }
        delay(2);
    }
}

MOT::~MOT() {
    delete dshot1;
    delete dshot2;
}

void MOT::update(const ACTb& actb) {
    static uint32_t print_counter = 0;
    print_counter++;
    if (print_counter >= 500) {
        print_counter = 0;
        Serial.print("MOT STATUS: ");
        if (dshot1 == nullptr || dshot1->initError()) {
            Serial.print("D1:FAIL ");
        } else {
            Serial.print("D1:OK ");
        }
        if (dshot2 == nullptr || dshot2->initError()) {
            Serial.print("D2:FAIL ");
        } else {
            Serial.print("D2:OK ");
        }
        Serial.println();
    }

    if (dshot1 != nullptr && !dshot1->initError()) {
        float m1_frac = std::clamp(actb.m1_frac, 0.0f, 1.0f);
        float m2_frac = std::clamp(actb.m2_frac, 0.0f, 1.0f);
        uint16_t throttles[4] = {
            static_cast<uint16_t>(m1_frac * 2000.0f),
            static_cast<uint16_t>(m2_frac * 2000.0f),
            0,
            0
        };
        dshot1->sendThrottles(throttles);
    }

    if (dshot2 != nullptr && !dshot2->initError()) {
        float m3_frac = std::clamp(actb.m3_frac, 0.0f, 1.0f);
        float m4_frac = std::clamp(actb.m4_frac, 0.0f, 1.0f);
        uint16_t throttles[4] = {
            static_cast<uint16_t>(m3_frac * 2000.0f),
            static_cast<uint16_t>(m4_frac * 2000.0f),
            0,
            0
        };
        dshot2->sendThrottles(throttles);
    }
}
