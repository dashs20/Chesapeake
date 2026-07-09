#include "CTL.hpp"

CTL::CTL(GNCc cfg) 
    : rate_controller(cfg.ctlc.rate), 
      angle_controller(cfg.ctlc.angle), 
      cfg_data(cfg), 
      time_accumulator_s(0.0f), 
      target_rates(Eigen::Vector3f::Zero()) {}

GNCb CTL::update(GNCb gnc) {
    if (gnc.vsmb.state == STATE::DISARMED) {
        rate_controller.reset();
        angle_controller.reset();
    }

    time_accumulator_s += cfg_data.dt_s;

    if (gnc.vsmb.att_mode == ATT_MODE::RATE) {
        Eigen::Vector3f setpoint = gnc.guib.omega_body_radps;
        Eigen::Vector3f measurement = gnc.navb.omega_body_radps;

        gnc.ctlb.axes_effort_frac = rate_controller.update(setpoint, measurement);
    } else if (gnc.vsmb.att_mode == ATT_MODE::ANGLE) {
        if (time_accumulator_s >= cfg_data.ctlc.angle_loop_dt_s) {
            time_accumulator_s = 0.0f;

            float roll_error = gnc.guib.euler_bodyz2up_rad.x() - gnc.navb.euler_bodyz2up_rad.x();
            float pitch_error = gnc.guib.euler_bodyz2up_rad.y() - gnc.navb.euler_bodyz2up_rad.y();

            Eigen::Vector3f euler_error(roll_error, pitch_error, 0.0f);
            Eigen::Vector3f target_rates_xy = angle_controller.update(euler_error, Eigen::Vector3f::Zero());

            target_rates.x() = target_rates_xy.x();
            target_rates.y() = target_rates_xy.y();
            target_rates.z() = gnc.guib.omega_body_radps.z();
        }

        Eigen::Vector3f measurement = gnc.navb.omega_body_radps;
        gnc.ctlb.axes_effort_frac = rate_controller.update(target_rates, measurement);
    }

    return gnc;
}
