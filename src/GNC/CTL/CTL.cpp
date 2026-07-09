#include "CTL.hpp"
#include "../UTIL/quat_util.hpp"

CTL::CTL(GNCc cfg) 
    : rate_controller(cfg.rate), 
      angle_controller(cfg.angle), 
      cfg_data(cfg), 
      time_accumulator_s(0.0f), 
      target_rates(Eigen::Vector3f::Zero()) {}

GNCb CTL::update(GNCb gnc) {
    time_accumulator_s += cfg_data.dt_s;

    if (gnc.vsmb.control_mode == CONTROL_MODE::RATE) {
        Eigen::Vector3f setpoint = gnc.guib.omega_body_radps;
        Eigen::Vector3f measurement = gnc.navb.omega_body_radps;

        gnc.ctlb.axes_effort_frac = rate_controller.update(setpoint, measurement);
    } 
    else if (gnc.vsmb.control_mode == CONTROL_MODE::ANGLE) {
        if (time_accumulator_s >= cfg_data.ctlc.angle_loop_dt_s) {
            time_accumulator_s = 0.0f;

            Eigen::Quaternionf q_body2body_des = gnc.navb.q_earth2body.inverse() * gnc.guib.q_earth2body;

            Eigen::Vector3f euler_error = quat_to_euler(q_body2body_des);

            target_rates = angle_controller.update(euler_error, Eigen::Vector3f::Zero());
        }

        Eigen::Vector3f measurement = gnc.navb.omega_body_radps;
        gnc.ctlb.axes_effort_frac = rate_controller.update(target_rates, measurement);
    }

    return gnc;
}
