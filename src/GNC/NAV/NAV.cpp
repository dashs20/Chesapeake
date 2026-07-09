#include "NAV.hpp"

NAV::NAV(GNCc cfg) : cfg_data(cfg) {
    dt_s = cfg.navc.dt_s;
    x = cfg.navc.x0;
    ukf.initialize(x);
    prev_omega_body_radps = Eigen::Vector3f::Zero();
}

NAVb NAV::update(const GNCb& gnc) {
    IMU_Compensated compensated_imu = compensate_imu(gnc);

    Eigen::Vector3d accel_CG_double_mps2 = compensated_imu.accel_CG_mps2.cast<double>();
    Eigen::Vector3d omega_body_double_radps = compensated_imu.omega_body_radps.cast<double>();
    x = ukf.update(x, dt_s, accel_CG_double_mps2, omega_body_double_radps);

    NAVb navb;
    navb.omega_body_radps = x.tail<3>().cast<float>();
    navb.q_earth2body = ukf.getOrientation().cast<float>();
    navb.up_body_hat = (navb.q_earth2body * Eigen::Vector3f::UnitZ()).normalized();

    float pitch_rad = std::asin(-navb.up_body_hat.x());
    float roll_rad = std::atan2(navb.up_body_hat.y(), navb.up_body_hat.z());
    navb.euler_bodyz2up_rad = Eigen::Vector2f(roll_rad, pitch_rad);

    return navb;
}

IMU_Compensated NAV::compensate_imu(const GNCb& gnc) {
    Eigen::Vector3f accel_raw_mps2 = gnc.halb.imub.accel_body_mps2;
    Eigen::Vector3f omega_raw_radps = gnc.halb.imub.omega_body_radps;

    Eigen::Vector3f accel_body_raw_mps2 = cfg_data.navc.q_IMU2body * accel_raw_mps2;
    Eigen::Vector3f omega_body_radps = cfg_data.navc.q_IMU2body * omega_raw_radps;

    Eigen::Vector3f r_body_m = cfg_data.navc.r_IMU2CG_mm * 0.001f;
    Eigen::Vector3f omega_dot_radps2 = Eigen::Vector3f::Zero();
    if (dt_s > 0.0) {
        omega_dot_radps2 = (omega_body_radps - prev_omega_body_radps) / static_cast<float>(dt_s);
    }
    prev_omega_body_radps = omega_body_radps;

    Eigen::Vector3f centrifugal_accel_mps2 = omega_body_radps.cross(omega_body_radps.cross(r_body_m));
    Eigen::Vector3f euler_accel_mps2 = omega_dot_radps2.cross(r_body_m);
    Eigen::Vector3f accel_CG_mps2 = accel_body_raw_mps2 - centrifugal_accel_mps2 - euler_accel_mps2;

    IMU_Compensated output;
    output.accel_CG_mps2 = accel_CG_mps2;
    output.omega_body_radps = omega_body_radps;
    return output;
}
