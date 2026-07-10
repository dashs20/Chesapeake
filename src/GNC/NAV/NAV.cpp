#include "NAV.hpp"

NAV::NAV(GNCc cfg) : cfg_data(cfg) {}

NAVb NAV::update(const GNCb& gnc) {
    IMU_Compensated compensated_imu = compensate_imu(gnc);

    NAVb navb;
    navb.omega_body_radps = compensated_imu.omega_body_radps;
    navb.q_earth2body = Eigen::Quaternionf::Identity();
    navb.up_body_hat = Eigen::Vector3f::UnitZ();
    navb.euler_bodyz2up_rad = Eigen::Vector2f::Zero();

    return navb;
}

IMU_Compensated NAV::compensate_imu(const GNCb& gnc) {
    Eigen::Vector3f accel_raw_mps2 = gnc.halb.imub.accel_body_mps2;
    Eigen::Vector3f omega_raw_radps = gnc.halb.imub.omega_body_radps;

    accel_raw_mps2 -= cfg_data.navc.accel_bias;
    omega_raw_radps -= cfg_data.navc.gyro_bias;

    Eigen::Vector3f accel_body_raw_mps2 = cfg_data.navc.q_IMU2body * accel_raw_mps2;
    Eigen::Vector3f omega_body_radps = cfg_data.navc.q_IMU2body * omega_raw_radps;

    IMU_Compensated output;
    output.accel_CG_mps2 = accel_body_raw_mps2;
    output.omega_body_radps = omega_body_radps;
    return output;
}
