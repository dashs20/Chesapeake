#include "CTL.hpp"

CTL::CTL(GNCc cfg) 
    : rate_controller(cfg.ctlc.rate), 
      angle_controller(cfg.ctlc.angle), 
      cfg_data(cfg), 
      time_accumulator_s(0.0f), 
      target_rates_radps(Eigen::Vector3f::Zero()) {}

CTLb CTL::update(const GNCb& gnc) {
    if (gnc.vsmb.state == STATE::DISARMED) {
        rate_controller.reset();
        angle_controller.reset();
    }

    time_accumulator_s += cfg_data.dt_s;

    CTLb ctlb;

    if (gnc.vsmb.att_mode == ATT_MODE::RATE) {
        Eigen::Vector3f omega_body_setpoint_radps = gnc.guib.omega_body_radps;
        Eigen::Vector3f omega_body_measurement_radps = gnc.navb.omega_body_radps;

        ctlb.axes_effort_frac = rate_controller.update(omega_body_setpoint_radps, omega_body_measurement_radps);
    } else if (gnc.vsmb.att_mode == ATT_MODE::ANGLE) {
        if (time_accumulator_s >= cfg_data.ctlc.angle_loop_dt_s) {
            time_accumulator_s = 0.0f;

            float roll_error_rad = gnc.guib.euler_bodyz2up_rad.x() - gnc.navb.euler_bodyz2up_rad.x();
            float pitch_error_rad = gnc.guib.euler_bodyz2up_rad.y() - gnc.navb.euler_bodyz2up_rad.y();

            Eigen::Vector3f euler_error_rad(roll_error_rad, pitch_error_rad, 0.0f);
            Eigen::Vector3f target_rates_xy_radps = angle_controller.update(euler_error_rad, Eigen::Vector3f::Zero());

            target_rates_radps.x() = target_rates_xy_radps.x();
            target_rates_radps.y() = target_rates_xy_radps.y();
            target_rates_radps.z() = gnc.guib.omega_body_radps.z();
        }

        Eigen::Vector3f omega_body_measurement_radps = gnc.navb.omega_body_radps;
        ctlb.axes_effort_frac = rate_controller.update(target_rates_radps, omega_body_measurement_radps);
    }

    return ctlb;
}
