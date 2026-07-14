#include "ALLOC.hpp"
#include <algorithm>

ALLOC::ALLOC(GNCc cfg) : cfg_data(cfg) {}

ACTb ALLOC::update(const ALLb& allb) {
    ACTb actb;
    if (!allb.gncb.vsmb.armed) {
        actb.m1_frac = 0.0f;
        actb.m2_frac = 0.0f;
        actb.m3_frac = 0.0f;
        actb.m4_frac = 0.0f;

        actb.s1_deg = cfg_data.allocc.ser_default_ang_deg;
        actb.s2_deg = cfg_data.allocc.ser_default_ang_deg;
        actb.s3_deg = cfg_data.allocc.ser_default_ang_deg;
        actb.s4_deg = cfg_data.allocc.ser_default_ang_deg;

        actb = clamp_actuators(allb, actb, 0.0f);
    } else {
        if (allb.gncb.vsmb.mode == FLIGHT_MODE::ACT_TEST) {
            actb = allb.cfg_appb.act_test.commands;
            actb = clamp_actuators(allb, actb, 0.0f);
        } else {
            actb = run_allocator(allb);
            actb = clamp_actuators(allb, actb, cfg_data.allocc.min_motor_frac);
        }
    }
    actb.LED_blink_Hz = determine_blink_hz(allb.gncb.vsmb.armed, allb.gncb.vsmb.mode);
    return actb;
}

ACTb ALLOC::run_allocator(const ALLb& allb) {
    if (cfg_data.allocator == ALLOCATOR::QUAD) {
        return allocate_quad(allb);
    } else {
        ACTb actb;
        actb.m1_frac = 0.0f;
        actb.m2_frac = 0.0f;
        actb.m3_frac = 0.0f;
        actb.m4_frac = 0.0f;

        actb.s1_deg = cfg_data.allocc.ser_default_ang_deg;
        actb.s2_deg = cfg_data.allocc.ser_default_ang_deg;
        actb.s3_deg = cfg_data.allocc.ser_default_ang_deg;
        actb.s4_deg = cfg_data.allocc.ser_default_ang_deg;
        return actb;
    }
}

ACTb ALLOC::allocate_quad(const ALLb& allb) {
    float throttle_frac = allb.halb.rcrxb.thr_frac;
    float roll_effort_frac = allb.gncb.ctlb.axes_effort_frac.x();
    float pitch_effort_frac = allb.gncb.ctlb.axes_effort_frac.y();
    float yaw_effort_frac = allb.gncb.ctlb.axes_effort_frac.z();

    ACTb actb;
    actb.m1_frac = throttle_frac - roll_effort_frac - pitch_effort_frac - yaw_effort_frac;
    actb.m2_frac = throttle_frac - roll_effort_frac + pitch_effort_frac + yaw_effort_frac;
    actb.m3_frac = throttle_frac + roll_effort_frac - pitch_effort_frac + yaw_effort_frac;
    actb.m4_frac = throttle_frac + roll_effort_frac + pitch_effort_frac - yaw_effort_frac;

    actb.s1_deg = cfg_data.allocc.ser_default_ang_deg;
    actb.s2_deg = cfg_data.allocc.ser_default_ang_deg;
    actb.s3_deg = cfg_data.allocc.ser_default_ang_deg;
    actb.s4_deg = cfg_data.allocc.ser_default_ang_deg;

    return actb;
}

ACTb ALLOC::clamp_actuators(const ALLb& allb, ACTb actb, float min_motor_frac) {
    actb.m1_frac = std::max(min_motor_frac, std::min(cfg_data.allocc.max_motor_frac, actb.m1_frac));
    actb.m2_frac = std::max(min_motor_frac, std::min(cfg_data.allocc.max_motor_frac, actb.m2_frac));
    actb.m3_frac = std::max(min_motor_frac, std::min(cfg_data.allocc.max_motor_frac, actb.m3_frac));
    actb.m4_frac = std::max(min_motor_frac, std::min(cfg_data.allocc.max_motor_frac, actb.m4_frac));

    actb.s1_deg = std::max(cfg_data.allocc.ser_min_ang_deg, std::min(cfg_data.allocc.ser_max_ang_deg, actb.s1_deg));
    actb.s2_deg = std::max(cfg_data.allocc.ser_min_ang_deg, std::min(cfg_data.allocc.ser_max_ang_deg, actb.s2_deg));
    actb.s3_deg = std::max(cfg_data.allocc.ser_min_ang_deg, std::min(cfg_data.allocc.ser_max_ang_deg, actb.s3_deg));
    actb.s4_deg = std::max(cfg_data.allocc.ser_min_ang_deg, std::min(cfg_data.allocc.ser_max_ang_deg, actb.s4_deg));

    return actb;
}

float ALLOC::determine_blink_hz(bool armed, FLIGHT_MODE mode) {
    if (!armed) {
        return cfg_data.allocc.blink_hz_disarmed;
    }
    switch (mode) {
        case FLIGHT_MODE::RATE:
            return cfg_data.allocc.blink_hz_rate;
        case FLIGHT_MODE::ANGLE:
            return cfg_data.allocc.blink_hz_angle;
        case FLIGHT_MODE::ACT_TEST:
            return 5.0f;
        default:
            return 0.0f;
    }
}
