#include "GUI.hpp"
#include <cmath>

GUI::GUI(GNCc cfg) : cfg_data(cfg) {}

float GUI::apply_expo(float input, float expo) {
    return expo * std::pow(input, 3.0f) + (1.0f - expo) * input;
}

GNCb GUI::update(GNCb gnc) {
    float roll_expo = apply_expo(gnc.halb.stick.roll_frac, cfg_data.guic.expoc.roll);
    float pitch_expo = apply_expo(gnc.halb.stick.pitch_frac, cfg_data.guic.expoc.pitch);
    float yaw_expo = apply_expo(gnc.halb.stick.yaw_frac, cfg_data.guic.expoc.yaw);

    if (gnc.vsmb.att_mode == ATT_MODE::RATE) {
        gnc.guib.omega_body_radps.x() = roll_expo * cfg_data.guic.max_rate_radps;
        gnc.guib.omega_body_radps.y() = pitch_expo * cfg_data.guic.max_rate_radps;
        gnc.guib.omega_body_radps.z() = yaw_expo * cfg_data.guic.max_rate_radps;
        gnc.guib.euler_bodyz2up_rad = Eigen::Vector2f::Zero();
    } else if (gnc.vsmb.att_mode == ATT_MODE::ANGLE) {
        gnc.guib.euler_bodyz2up_rad.x() = roll_expo * cfg_data.guic.max_angle_rad;
        gnc.guib.euler_bodyz2up_rad.y() = pitch_expo * cfg_data.guic.max_angle_rad;
        gnc.guib.omega_body_radps.z() = yaw_expo * cfg_data.guic.max_rate_radps;
        gnc.guib.omega_body_radps.x() = 0.0f;
        gnc.guib.omega_body_radps.y() = 0.0f;
    }

    return gnc;
}
