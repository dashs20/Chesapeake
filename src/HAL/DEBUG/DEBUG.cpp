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

    Serial.printf("$DBG,%.4f,%.4f\n",
                  allb_km1.halb.execution_time_ms,
                  allb_km1.gnc_time_ms);

    // float ax = allb_km1.halb.imub.accel_body_mps2.x();
    // float ay = allb_km1.halb.imub.accel_body_mps2.y();
    // float az = allb_km1.halb.imub.accel_body_mps2.z();
    // float gx = allb_km1.halb.imub.omega_body_radps.x() * 57.29577951f;
    // float gy = allb_km1.halb.imub.omega_body_radps.y() * 57.29577951f;
    // float gz = allb_km1.halb.imub.omega_body_radps.z() * 57.29577951f;

    // Eigen::Vector3f up = allb_km1.navb.up_body_hat;
    // float pitch_abs = std::asin(-up.x()) * 57.29577951f;
    // float roll_abs = std::atan2(up.y(), up.z()) * 57.29577951f;

    // float roll_lib = allb_km1.navb.euler_bodyz2up_rad.x() * 57.29577951f;
    // float pitch_lib = allb_km1.navb.euler_bodyz2up_rad.y() * 57.29577951f;

    // Serial.printf("$DBG,%+07.2f,%+07.2f,%+07.2f,%+07.2f,%+07.2f,%+07.2f,%+07.2f,%+07.2f,%+07.2f,%+07.2f\n",
    //               ax, ay, az, gx, gy, gz, roll_lib, pitch_lib, roll_abs, pitch_abs);

    // Serial.printf("$DBG,%+06.2f,%+06.2f,%+06.2f\n", gx, gy, gz);
}
