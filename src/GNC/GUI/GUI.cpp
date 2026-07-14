#include "GUI.hpp"
#include <cmath>

GUI::GUI(GNCc cfg) : cfg_data(cfg) {}

float GUI::apply_expo(float input_frac, float expo_frac) {
    return expo_frac * std::pow(input_frac, 3.0f) + (1.0f - expo_frac) * input_frac;
}

GUIb GUI::update(const ALLb& allb) {
    float roll_expo_frac = apply_expo(allb.halb.rcrxb.roll_frac, cfg_data.guic.expoc.roll);
    float pitch_expo_frac = apply_expo(allb.halb.rcrxb.pitch_frac, cfg_data.guic.expoc.pitch);
    float yaw_expo_frac = apply_expo(allb.halb.rcrxb.yaw_frac, cfg_data.guic.expoc.yaw);

    GUIb guib;
    switch (allb.gncb.vsmb.mode) {
        case FLIGHT_MODE::ACT_TEST:
            guib.omega_body_radps = Eigen::Vector3f::Zero();
            guib.euler_bodyz2up_rad = Eigen::Vector2f::Zero();
            break;
        case FLIGHT_MODE::RATE:
            guib.omega_body_radps.x() = roll_expo_frac * cfg_data.guic.max_rate_radps;
            guib.omega_body_radps.y() = pitch_expo_frac * cfg_data.guic.max_rate_radps;
            guib.omega_body_radps.z() = yaw_expo_frac * cfg_data.guic.max_rate_radps;
            guib.euler_bodyz2up_rad = Eigen::Vector2f::Zero();
            break;
        case FLIGHT_MODE::ANGLE:
            guib.euler_bodyz2up_rad.x() = roll_expo_frac * cfg_data.guic.max_angle_rad;
            guib.euler_bodyz2up_rad.y() = pitch_expo_frac * cfg_data.guic.max_angle_rad;
            guib.omega_body_radps.x() = 0.0f;
            guib.omega_body_radps.y() = 0.0f;
            guib.omega_body_radps.z() = yaw_expo_frac * cfg_data.guic.max_rate_radps;
            break;
        default:
            guib.omega_body_radps = Eigen::Vector3f::Zero();
            guib.euler_bodyz2up_rad = Eigen::Vector2f::Zero();
            break;
    }

    return guib;
}
