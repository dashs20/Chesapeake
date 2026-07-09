#pragma once
#include <Eigen/Dense>

// HAL BUS ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
struct IMU { // IMU sub bus
    Eigen::Vector3f omega_body_radps;
    Eigen::Vector3f accel_body_mps2;
};

struct STICK { // STICK sub bus
    float arm_frac;
    float mode_frac;
    float thr_frac;
    float roll_frac;
    float pitch_frac;
    float yaw_frac;
};

struct HALb {
    IMU imu;
    STICK stick;
};
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// VSM BUS ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
enum class STATE{IDLE, RATE, ANGLE, GPS_HOLD};
enum class CONTROL_MODE{RATE, ANGLE};

struct VSMb{
    STATE state;
    CONTROL_MODE control_mode;
};
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// ACT BUS ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
struct ACTb{
    float m1_frac;
    float m2_frac;
    float m3_frac;
    float m4_frac;
    float s1_deg;
    float s2_deg;
    float s3_deg;
    float s4_deg;
};
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// NAV BUS ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
struct NAVb{ // NAV bus
    Eigen::Vector3f omega_body_radps;
    Eigen::Quaternionf q_earth2body; // estimated orientation with respect to "earth" frame; we don't have a compass, so we don't know where north is.
};
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// CTL BUS ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
struct CTLb{ // CTL bus
    Eigen::Vector3f axes_effort_frac; // unitless rotation effort; maps differently to angle mode and rate mode.
};
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// GUI BUS ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
struct GUIb {
    Eigen::Quaternionf q_earth2body;     // Desired orientation (rotation from earth to desired body frame)
    Eigen::Vector3f omega_body_radps;    // Commanded body rates for rate mode
};
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// GNC MASTER BUS ~--~--~--~--~--~--~--~--~--~--~--~--~--~--~--~--~-
struct GNCb{
    HALb halb;
    VSMb vsmb;
    ACTb actb;
    NAVb navb;
    CTLb ctlb;
    GUIb guib;
};
// --~--~--~--~--~--~--~--~--~--~--~--~--~--~--~--~--~--~--~--~--~-