#include "NAV.hpp"

NAV::NAV(GNCc cfg) : cfg_data(cfg) {
    dt_s = 1.0 / static_cast<double>(cfg.looprate_hz);
    filter = new IMUfilter(dt_s, static_cast<double>(cfg.navc.gyro_error_degps));
}

NAV::~NAV() {
    delete filter;
}

void NAV::reset() {
    filter->reset();
}

NAVb NAV::update(const ALLb& allb) {
    IMU_Compensated compensated_imu = compensate_imu(allb);

    filter->updateFilter(
        static_cast<double>(compensated_imu.omega_body_radps.x()),
        static_cast<double>(compensated_imu.omega_body_radps.y()),
        static_cast<double>(compensated_imu.omega_body_radps.z()),
        static_cast<double>(compensated_imu.accel_CG_mps2.x()),
        static_cast<double>(compensated_imu.accel_CG_mps2.y()),
        static_cast<double>(compensated_imu.accel_CG_mps2.z())
    );

    filter->computeEuler();

    NAVb navb;
    navb.omega_body_radps = compensated_imu.omega_body_radps;
    navb.q_earth2body = Eigen::Quaternionf(
        static_cast<float>(filter->getQ1()),
        static_cast<float>(-filter->getQ2()),
        static_cast<float>(-filter->getQ3()),
        static_cast<float>(-filter->getQ4())
    ).normalized();
    navb.up_body_hat = (navb.q_earth2body * Eigen::Vector3f::UnitZ()).normalized();
    navb.euler_bodyz2up_rad = Eigen::Vector2f(
        static_cast<float>(filter->getRoll()),
        static_cast<float>(filter->getPitch())
    );

    return navb;
}
 
IMU_Compensated NAV::compensate_imu(const ALLb& allb) {
    Eigen::Vector3f accel_raw_mps2 = allb.halb.imub.accel_body_mps2;
    Eigen::Vector3f omega_raw_radps = allb.halb.imub.omega_body_radps;
 
    Eigen::Vector3f accel_body_raw_mps2 = cfg_data.navc.q_IMU2body.conjugate() * accel_raw_mps2;
    Eigen::Vector3f omega_body_radps = cfg_data.navc.q_IMU2body.conjugate() * omega_raw_radps;
 
    accel_body_raw_mps2 -= cfg_data.navc.imu_calc.accel_bias;
    omega_body_radps -= cfg_data.navc.imu_calc.gyro_bias;
 
    IMU_Compensated output;
    output.accel_CG_mps2 = accel_body_raw_mps2;
    output.omega_body_radps = omega_body_radps;
    return output;
}
