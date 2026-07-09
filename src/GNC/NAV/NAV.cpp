#include "NAV.hpp"

NAV::NAV(GNCc cfg) {
    dt_s = cfg.navc.dt_s;
    x = cfg.navc.x0;
    ukf.initialize(x);
}

GNCb NAV::update(GNCb gnc) {
    Eigen::Vector3d accel = gnc.halb.imu.accel_body_mps2.cast<double>();
    Eigen::Vector3d gyro = gnc.halb.imu.omega_body_radps.cast<double>();

    x = ukf.update(x, dt_s, accel, gyro);

    gnc.navb.omega_body_radps = x.tail<3>().cast<float>();
    gnc.navb.q_earth2body = ukf.getOrientation().cast<float>();
    
    gnc.navb.up_body_hat = (gnc.navb.q_earth2body * Eigen::Vector3f::UnitZ()).normalized();
    float pitch_rad = std::asin(-gnc.navb.up_body_hat.x());
    float roll_rad = std::atan2(gnc.navb.up_body_hat.y(), gnc.navb.up_body_hat.z());
    gnc.navb.euler_bodyz2up_rad = Eigen::Vector2f(roll_rad, pitch_rad);

    return gnc;
}
