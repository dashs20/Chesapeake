#include "NAV.hpp"

NAV::NAV(GNCc cfg) : cfg_data(cfg) {
    dt_s = 1.0 / static_cast<double>(cfg.looprate_hz);
    prev_omega_body_radps = Eigen::Vector3f::Zero();
}

NAVb NAV::update(const GNCb& gnc) {
    IMU_Compensated compensated_imu = compensate_imu(gnc);

    NAVb navb;
    navb.omega_body_radps = Eigen::Vector3f::Zero();
    navb.q_earth2body = Eigen::Quaternionf::Identity();
    navb.up_body_hat = Eigen::Vector3f::UnitZ();
    navb.euler_bodyz2up_rad = Eigen::Vector2f::Zero();

    return navb;
}

IMU_Compensated NAV::compensate_imu(const GNCb& gnc) {
    Eigen::Vector3f accel_raw_mps2 = gnc.halb.imub.accel_body_mps2;
    Eigen::Vector3f omega_raw_radps = gnc.halb.imub.omega_body_radps;

    // Apply IMU calibration biases
    accel_raw_mps2 -= cfg_data.navc.accel_bias;
    omega_raw_radps -= cfg_data.navc.gyro_bias;

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
