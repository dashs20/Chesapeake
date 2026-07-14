#include "SER.hpp"
#include <algorithm>

SER::SER(SERc cfg) : serc(cfg) {
    servos[0].attach(serc.s1_pin, static_cast<int>(serc.min_us), static_cast<int>(serc.max_us));
    servos[1].attach(serc.s2_pin, static_cast<int>(serc.min_us), static_cast<int>(serc.max_us));
    servos[2].attach(serc.s3_pin, static_cast<int>(serc.min_us), static_cast<int>(serc.max_us));
    servos[3].attach(serc.s4_pin, static_cast<int>(serc.min_us), static_cast<int>(serc.max_us));
}

SER::~SER() {
    for (uint8_t i = 0; i < 4; ++i) {
        servos[i].detach();
    }
}

void SER::update(const ACTb& actb) {
    servos[0].write(static_cast<int>(std::clamp(actb.s1_deg, 0.0f, 180.0f)));
    servos[1].write(static_cast<int>(std::clamp(actb.s2_deg, 0.0f, 180.0f)));
    servos[2].write(static_cast<int>(std::clamp(actb.s3_deg, 0.0f, 180.0f)));
    servos[3].write(static_cast<int>(std::clamp(actb.s4_deg, 0.0f, 180.0f)));
}
