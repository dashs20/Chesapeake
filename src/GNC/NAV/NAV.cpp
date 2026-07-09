#include "NAV.hpp"

NAV::NAV(GNCc cfg) {
    dt_s = cfg.navc.dt_s;
    x = cfg.navc.x0;
    ukf.initialize(x);
}

NAVb NAV::update(const GNCb& gnc) {
    Eigen::Vector3d accel = gnc.halb.imu.accel_body_mps2.cast<double>();
    Eigen::Vector3d gyro = gnc.halb.imu.omega_body_radps.cast<double>();

    x = ukf.update(x, dt_s, accel, gyro);

    NAVb navb;
    navb.omega_body_radps = x.tail<3>().cast<float>();
    navb.q_earth2body = ukf.getOrientation().cast<float>();
    
    navb.up_body_hat = (navb.q_earth2body * Eigen::Vector3f::UnitZ()).normalized();
    float pitch_rad = std::asin(-navb.up_body_hat.x());
    float roll_rad = std::atan2(navb.up_body_hat.y(), navb.up_body_hat.z());
    navb.euler_bodyz2up_rad = Eigen::Vector2f(roll_rad, pitch_rad);

    return navb;
}
