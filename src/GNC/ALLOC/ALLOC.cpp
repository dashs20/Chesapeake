#include "ALLOC.hpp"
#include <algorithm>

ALLOC::ALLOC(GNCc cfg) : cfg_data(cfg) {}

ACTb ALLOC::update(const GNCb& gnc) {
    if (gnc.vsmb.state == STATE::DISARMED) {
        ACTb actb;
        actb.m1_frac = 0.0f;
        actb.m2_frac = 0.0f;
        actb.m3_frac = 0.0f;
        actb.m4_frac = 0.0f;

        actb.s1_deg = cfg_data.allocc.servo_default_ang_deg;
        actb.s2_deg = cfg_data.allocc.servo_default_ang_deg;
        actb.s3_deg = cfg_data.allocc.servo_default_ang_deg;
        actb.s4_deg = cfg_data.allocc.servo_default_ang_deg;

        return clamp_actuators(gnc, actb);
    } else {
        ACTb actb = run_allocator(gnc);
        return clamp_actuators(gnc, actb);
    }
}

ACTb ALLOC::run_allocator(const GNCb& gnc) {
    if (cfg_data.allocator == ALLOCATOR::QUAD) {
        return allocate_quad(gnc);
    } else {
        ACTb actb;
        actb.m1_frac = 0.0f;
        actb.m2_frac = 0.0f;
        actb.m3_frac = 0.0f;
        actb.m4_frac = 0.0f;

        actb.s1_deg = cfg_data.allocc.servo_default_ang_deg;
        actb.s2_deg = cfg_data.allocc.servo_default_ang_deg;
        actb.s3_deg = cfg_data.allocc.servo_default_ang_deg;
        actb.s4_deg = cfg_data.allocc.servo_default_ang_deg;
        return actb;
    }
}

ACTb ALLOC::allocate_quad(const GNCb& gnc) {
    float throttle = gnc.halb.stick.thr_frac;
    float roll_effort = gnc.ctlb.axes_effort_frac.x();
    float pitch_effort = gnc.ctlb.axes_effort_frac.y();
    float yaw_effort = gnc.ctlb.axes_effort_frac.z();

    ACTb actb;
    actb.m1_frac = throttle - roll_effort + pitch_effort - yaw_effort;
    actb.m2_frac = throttle - roll_effort - pitch_effort + yaw_effort;
    actb.m3_frac = throttle + roll_effort + pitch_effort + yaw_effort;
    actb.m4_frac = throttle + roll_effort - pitch_effort - yaw_effort;

    actb.s1_deg = cfg_data.allocc.servo_default_ang_deg;
    actb.s2_deg = cfg_data.allocc.servo_default_ang_deg;
    actb.s3_deg = cfg_data.allocc.servo_default_ang_deg;
    actb.s4_deg = cfg_data.allocc.servo_default_ang_deg;

    return actb;
}

ACTb ALLOC::clamp_actuators(const GNCb& gnc, ACTb actb) {
    float minimum_motor_fraction = (gnc.vsmb.state == STATE::DISARMED) ? 0.0f : cfg_data.allocc.min_motor_frac;

    actb.m1_frac = std::max(minimum_motor_fraction, std::min(1.0f, actb.m1_frac));
    actb.m2_frac = std::max(minimum_motor_fraction, std::min(1.0f, actb.m2_frac));
    actb.m3_frac = std::max(minimum_motor_fraction, std::min(1.0f, actb.m3_frac));
    actb.m4_frac = std::max(minimum_motor_fraction, std::min(1.0f, actb.m4_frac));

    actb.s1_deg = std::max(cfg_data.allocc.servo_min_ang_deg, std::min(cfg_data.allocc.servo_max_ang_deg, actb.s1_deg));
    actb.s2_deg = std::max(cfg_data.allocc.servo_min_ang_deg, std::min(cfg_data.allocc.servo_max_ang_deg, actb.s2_deg));
    actb.s3_deg = std::max(cfg_data.allocc.servo_min_ang_deg, std::min(cfg_data.allocc.servo_max_ang_deg, actb.s3_deg));
    actb.s4_deg = std::max(cfg_data.allocc.servo_min_ang_deg, std::min(cfg_data.allocc.servo_max_ang_deg, actb.s4_deg));

    return actb;
}
