#include "CTL.hpp"
#include <cassert>

CTL::CTL(GNCc cfg) 
    : rate_controller([&]() {
          PID_3DOFc rate_cfg = cfg.ctlc.rate;
          float dt = 1.0f / static_cast<float>(cfg.looprate_hz);
          rate_cfg.roll.dt_s = dt;
          rate_cfg.pitch.dt_s = dt;
          rate_cfg.yaw.dt_s = dt;
          return rate_cfg;
      }()), 
      angle_controller([&]() {
          PID_3DOFc angle_cfg = cfg.ctlc.angle;
          float dt = 1.0f / static_cast<float>(cfg.ctlc.angle_loop_hz);
          angle_cfg.roll.dt_s = dt;
          angle_cfg.pitch.dt_s = dt;
          angle_cfg.yaw.dt_s = dt;
          return angle_cfg;
      }()), 
      cfg_data(cfg), 
      dt_s(1.0f / static_cast<float>(cfg.looprate_hz)),
      angle_loop_dt_s(1.0f / static_cast<float>(cfg.ctlc.angle_loop_hz)),
      time_accumulator_s(0.0f), 
      target_rates_radps(Eigen::Vector3f::Zero()) {
    assert(cfg.ctlc.angle_loop_hz < cfg.looprate_hz);
}

CTLb CTL::update(const ALLb& allb) {
    if (!allb.gncb.vsmb.armed || allb.gncb.vsmb.mode == FLIGHT_MODE::ACT_TEST) {
        rate_controller.reset();
        angle_controller.reset();
        return CTLb{Eigen::Vector3f::Zero()};
    }

    time_accumulator_s += dt_s;

    CTLb ctlb;

    switch (allb.gncb.vsmb.mode) {
        case FLIGHT_MODE::RATE: {
            Eigen::Vector3f omega_body_setpoint_radps = allb.gncb.guib.omega_body_radps;
            Eigen::Vector3f omega_body_measurement_radps = allb.gncb.navb.omega_body_radps;
            ctlb.axes_effort_frac = rate_controller.update(omega_body_setpoint_radps, omega_body_measurement_radps);
            break;
        }
        case FLIGHT_MODE::ANGLE: {
            if (time_accumulator_s >= angle_loop_dt_s) {
                time_accumulator_s = 0.0f;
                float roll_error_rad = allb.gncb.guib.euler_bodyz2up_rad.x() - allb.gncb.navb.euler_bodyz2up_rad.x();
                float pitch_error_rad = allb.gncb.guib.euler_bodyz2up_rad.y() - allb.gncb.navb.euler_bodyz2up_rad.y();
                Eigen::Vector3f euler_error_rad(roll_error_rad, pitch_error_rad, 0.0f);
                Eigen::Vector3f target_rates_xy_radps = angle_controller.update(euler_error_rad, Eigen::Vector3f::Zero());
                target_rates_radps.x() = target_rates_xy_radps.x();
                target_rates_radps.y() = target_rates_xy_radps.y();
                target_rates_radps.z() = allb.gncb.guib.omega_body_radps.z();
            }
            Eigen::Vector3f omega_body_measurement_radps = allb.gncb.navb.omega_body_radps;
            ctlb.axes_effort_frac = rate_controller.update(target_rates_radps, omega_body_measurement_radps);
            break;
        }
        default:
            ctlb.axes_effort_frac = Eigen::Vector3f::Zero();
            break;
    }

    return ctlb;
}
