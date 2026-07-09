#include "ALLOC.hpp"
#include <algorithm>

ALLOC::ALLOC(GNCc cfg) : cfg_data(cfg) {}

GNCb ALLOC::update(GNCb gnc) {
    switch (gnc.vsmb.state) {
        case STATE::DISARMED: {
            gnc.actb.m1_frac = 0.0f;
            gnc.actb.m2_frac = 0.0f;
            gnc.actb.m3_frac = 0.0f;
            gnc.actb.m4_frac = 0.0f;

            gnc.actb.s1_deg = cfg_data.allocc.servo_default_ang_deg;
            gnc.actb.s2_deg = cfg_data.allocc.servo_default_ang_deg;
            gnc.actb.s3_deg = cfg_data.allocc.servo_default_ang_deg;
            gnc.actb.s4_deg = cfg_data.allocc.servo_default_ang_deg;

            return clamp_actuators(gnc);
        }
        case STATE::RATE:
        case STATE::ANGLE:
        case STATE::GPS_HOLD: {
            gnc = run_allocator(gnc);
            return clamp_actuators(gnc);
        }
    }
    return gnc;
}

GNCb ALLOC::run_allocator(GNCb gnc) {
    if (cfg_data.allocator == ALLOCATOR::QUAD) {
        return allocate_quad(gnc);
    } else {
        gnc.actb.m1_frac = 0.0f;
        gnc.actb.m2_frac = 0.0f;
        gnc.actb.m3_frac = 0.0f;
        gnc.actb.m4_frac = 0.0f;

        gnc.actb.s1_deg = cfg_data.allocc.servo_default_ang_deg;
        gnc.actb.s2_deg = cfg_data.allocc.servo_default_ang_deg;
        gnc.actb.s3_deg = cfg_data.allocc.servo_default_ang_deg;
        gnc.actb.s4_deg = cfg_data.allocc.servo_default_ang_deg;
        return gnc;
    }
}

GNCb ALLOC::allocate_quad(GNCb gnc) {
    float throttle = gnc.halb.stick.thr_frac;
    float roll_effort = gnc.ctlb.axes_effort_frac.x();
    float pitch_effort = gnc.ctlb.axes_effort_frac.y();
    float yaw_effort = gnc.ctlb.axes_effort_frac.z();

    gnc.actb.m1_frac = throttle - roll_effort + pitch_effort - yaw_effort;
    gnc.actb.m2_frac = throttle - roll_effort - pitch_effort + yaw_effort;
    gnc.actb.m3_frac = throttle + roll_effort + pitch_effort + yaw_effort;
    gnc.actb.m4_frac = throttle + roll_effort - pitch_effort - yaw_effort;

    gnc.actb.s1_deg = cfg_data.allocc.servo_default_ang_deg;
    gnc.actb.s2_deg = cfg_data.allocc.servo_default_ang_deg;
    gnc.actb.s3_deg = cfg_data.allocc.servo_default_ang_deg;
    gnc.actb.s4_deg = cfg_data.allocc.servo_default_ang_deg;

    return gnc;
}

GNCb ALLOC::clamp_actuators(GNCb gnc) {
    float minimum_motor_fraction = (gnc.vsmb.state == STATE::DISARMED) ? 0.0f : cfg_data.allocc.min_motor_frac;

    gnc.actb.m1_frac = std::max(minimum_motor_fraction, std::min(1.0f, gnc.actb.m1_frac));
    gnc.actb.m2_frac = std::max(minimum_motor_fraction, std::min(1.0f, gnc.actb.m2_frac));
    gnc.actb.m3_frac = std::max(minimum_motor_fraction, std::min(1.0f, gnc.actb.m3_frac));
    gnc.actb.m4_frac = std::max(minimum_motor_fraction, std::min(1.0f, gnc.actb.m4_frac));

    gnc.actb.s1_deg = std::max(cfg_data.allocc.servo_min_ang_deg, std::min(cfg_data.allocc.servo_max_ang_deg, gnc.actb.s1_deg));
    gnc.actb.s2_deg = std::max(cfg_data.allocc.servo_min_ang_deg, std::min(cfg_data.allocc.servo_max_ang_deg, gnc.actb.s2_deg));
    gnc.actb.s3_deg = std::max(cfg_data.allocc.servo_min_ang_deg, std::min(cfg_data.allocc.servo_max_ang_deg, gnc.actb.s3_deg));
    gnc.actb.s4_deg = std::max(cfg_data.allocc.servo_min_ang_deg, std::min(cfg_data.allocc.servo_max_ang_deg, gnc.actb.s4_deg));

    return gnc;
}
