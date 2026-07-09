#pragma once
#include <Eigen/Dense>
#include "../HAL/bus.hpp"

// VSM BUS ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
enum class STATE{DISARMED, RATE, ANGLE};
enum class ATT_MODE{RATE, ANGLE};

struct VSMb{
    STATE state;
    ATT_MODE att_mode;
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
    float LED_blink_Hz;
};
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// NAV BUS ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
struct NAVb{ // NAV bus
    Eigen::Vector3f omega_body_radps;
    Eigen::Quaternionf q_earth2body; // estimated orientation with respect to "earth" frame; we don't have a compass, so we don't know where north is.
    Eigen::Vector3f up_body_hat;      // Estimated unit up-vector in the body frame
    Eigen::Vector2f euler_bodyz2up_rad; // Estimated Euler angles [roll, pitch] in radians
};
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// CTL BUS ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
struct CTLb{ // CTL bus
    Eigen::Vector3f axes_effort_frac; // unitless rotation effort; maps differently to angle mode and rate mode.
};
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// GUI BUS ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
struct GUIb {
    Eigen::Vector2f euler_bodyz2up_rad; // Commanded Euler angles [roll, pitch] in radians
    Eigen::Vector3f omega_body_radps;    // Commanded body rates in radians per second
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