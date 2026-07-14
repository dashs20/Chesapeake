#pragma once
#include <Eigen/Dense>
#include "../HAL/bus.hpp"
#include "../CFG_APP/bus.hpp"

// VSM BUS ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
enum class FLIGHT_MODE { RATE, ANGLE, ACT_TEST };

struct VSMb{
    bool armed;
    FLIGHT_MODE mode;
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

struct GNCb{
    VSMb vsmb;
    ACTb actb;
    NAVb navb;
    CTLb ctlb;
    GUIb guib;
    float gnc_time_ms;
    float time_nav_us;
    float time_ctl_us;
    float time_alloc_us;
};
// --~--~--~--~--~--~--~--~--~--~--~--~--~--~--~--~--~--~--~--~--~-

struct ALLb {
    HALb halb;
    GNCb gncb;
    CFG_APPb cfg_appb;
};