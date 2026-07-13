#include "DEBUG.hpp"

DEBUG::DEBUG(DEBUGc cfg) : debugc(cfg), counter(0) {}

DEBUG::~DEBUG() {}

void DEBUG::update(const ALLb& allb_km1) {
    if (!debugc.enabled) {
        return;
    }

    counter++;
    if (debugc.decimation > 0 && (counter % debugc.decimation) != 0) {
        return;
    }

    /*
    float ax = allb_km1.halb.imub.accel_body_mps2.x();
    float ay = allb_km1.halb.imub.accel_body_mps2.y();
    float az = allb_km1.halb.imub.accel_body_mps2.z();
    float gx = allb_km1.halb.imub.omega_body_radps.x() * 57.29577951f;
    float gy = allb_km1.halb.imub.omega_body_radps.y() * 57.29577951f;
    float gz = allb_km1.halb.imub.omega_body_radps.z() * 57.29577951f;

    Eigen::Vector3f up = allb_km1.gncb.navb.up_body_hat;
    float pitch_abs = std::asin(-up.x()) * 57.29577951f;
    float roll_abs = std::atan2(up.y(), up.z()) * 57.29577951f;

    float roll_lib = allb_km1.gncb.navb.euler_bodyz2up_rad.x() * 57.29577951f;
    float pitch_lib = allb_km1.gncb.navb.euler_bodyz2up_rad.y() * 57.29577951f;
    */

    float rx_roll = allb_km1.halb.rcrxb.roll_frac;
    float rx_pitch = allb_km1.halb.rcrxb.pitch_frac;
    float rx_yaw = allb_km1.halb.rcrxb.yaw_frac;
    float rx_thr = allb_km1.halb.rcrxb.thr_frac;
    float rx_arm = allb_km1.halb.rcrxb.arm_frac;
    float rx_mode = allb_km1.halb.rcrxb.mode_frac;

    Serial.printf("$DBG,%+07.2f,%+07.2f,%+07.2f,%+07.2f,%+07.2f,%+07.2f\n",
                  rx_roll, rx_pitch, rx_yaw, rx_thr, rx_arm, rx_mode);
}
