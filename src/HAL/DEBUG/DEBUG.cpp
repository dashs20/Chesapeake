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
    // This block translates the VSM STATE enum into a human-readable string (DISARMED, RATE, ANGLE)
    const char* state_str = "UNKNOWN";
    if (allb_km1.gncb.vsmb.state == STATE::DISARMED) {
        state_str = "DISARMED";
    } else if (allb_km1.gncb.vsmb.state == STATE::RATE) {
        state_str = "RATE";
    } else if (allb_km1.gncb.vsmb.state == STATE::ANGLE) {
        state_str = "ANGLE";
    }
    */

    /*
    float m1 = allb_km1.gncb.actb.m1_frac;
    float m2 = allb_km1.gncb.actb.m2_frac;
    float m3 = allb_km1.gncb.actb.m3_frac;
    float m4 = allb_km1.gncb.actb.m4_frac;

    Serial.printf("$DBG,STATE: %s | M1: %0.4f | M2: %0.4f | M3: %0.4f | M4: %0.4f\n",
                  state_str, m1, m2, m3, m4);
    */

    /*
    // This block displays the current vehicle state machine (VSM) status and all normalized RC receiver channel values (0.0 to 1.0)
    float thr = allb_km1.halb.rcrxb.thr_frac;
    float roll = allb_km1.halb.rcrxb.roll_frac;
    float pitch = allb_km1.halb.rcrxb.pitch_frac;
    float yaw = allb_km1.halb.rcrxb.yaw_frac;
    float arm = allb_km1.halb.rcrxb.arm_frac;
    float mode = allb_km1.halb.rcrxb.mode_frac;

    Serial.printf("$DBG,STATE: %s | THR: %0.4f | ROLL: %0.4f | PITCH: %0.4f | YAW: %0.4f | ARM: %0.4f | MODE: %0.4f\n",
                  state_str, thr, roll, pitch, yaw, arm, mode);
    */

    /*
    // This block displays the current vehicle state machine (VSM) status and the rotated IMU body rates in deg/s
    float imu_roll_degps = allb_km1.gncb.navb.omega_body_radps.x() * 57.29577951f;
    float imu_pitch_degps = allb_km1.gncb.navb.omega_body_radps.y() * 57.29577951f;
    float imu_yaw_degps = allb_km1.gncb.navb.omega_body_radps.z() * 57.29577951f;

    Serial.printf("$DBG,STATE: %d | IMU_ROLL: %+07.2f | IMU_PITCH: %+07.2f | IMU_YAW: %+07.2f\n",
                  static_cast<int>(allb_km1.gncb.vsmb.state), imu_roll_degps, imu_pitch_degps, imu_yaw_degps);
    */

    // This block displays the current vehicle state machine (VSM) status and the raw, un-rotated IMU gyroscope rates in deg/s (X, Y, Z axes)
    float raw_x_degps = allb_km1.halb.imub.omega_body_radps.x() * 57.29577951f;
    float raw_y_degps = allb_km1.halb.imub.omega_body_radps.y() * 57.29577951f;
    float raw_z_degps = allb_km1.halb.imub.omega_body_radps.z() * 57.29577951f;

    Serial.printf("$DBG,STATE: %d | IMU_X: %+07.2f | IMU_Y: %+07.2f | IMU_Z: %+07.2f\n",
                  static_cast<int>(allb_km1.gncb.vsmb.state), raw_x_degps, raw_y_degps, raw_z_degps);
}
