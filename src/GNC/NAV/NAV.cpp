#include "NAV.hpp"

NAV::NAV(GNCc cfg) : cfg_data(cfg) {
    dt_s = 1.0 / static_cast<double>(cfg.looprate_hz);
    filter = new IMUfilter(dt_s, static_cast<double>(cfg.navc.gyro_error_degps));
    is_calibrating = false;
    calibration_counter = 0;
    sum_ax = 0.0f;
    sum_ay = 0.0f;
    sum_az = 0.0f;
    sum_gx = 0.0f;
    sum_gy = 0.0f;
    sum_gz = 0.0f;
    cal_feedback.calibration_done = false;
    cal_feedback.is_calibrating = false;
    cal_feedback.calibration_progress_frac = 0.0f;
    cal_feedback.accel_bias_x = 0.0f;
    cal_feedback.accel_bias_y = 0.0f;
    cal_feedback.accel_bias_z = 0.0f;
    cal_feedback.gyro_bias_x = 0.0f;
    cal_feedback.gyro_bias_y = 0.0f;
    cal_feedback.gyro_bias_z = 0.0f;
}

NAV::~NAV() {
    delete filter;
}

void NAV::reset() {
    filter->reset();
}

NAVb NAV::update(const ALLb& allb) {
    if (allb.cfg_appb.calibrate_requested && !is_calibrating) {
        sum_ax = 0.0f;
        sum_ay = 0.0f;
        sum_az = 0.0f;
        sum_gx = 0.0f;
        sum_gy = 0.0f;
        sum_gz = 0.0f;
        calibration_counter = 0;
        is_calibrating = true;
        cal_feedback.calibration_done = false;
        cal_feedback.is_calibrating = true;
        cal_feedback.calibration_progress_frac = 0.0f;
    }

    IMU_Compensated compensated_imu = compensate_imu(allb);

    if (is_calibrating) {
        sum_ax += compensated_imu.accel_CG_mps2.x();
        sum_ay += compensated_imu.accel_CG_mps2.y();
        sum_az += compensated_imu.accel_CG_mps2.z();
        sum_gx += compensated_imu.omega_body_radps.x();
        sum_gy += compensated_imu.omega_body_radps.y();
        sum_gz += compensated_imu.omega_body_radps.z();
        calibration_counter++;
        cal_feedback.calibration_progress_frac = static_cast<float>(calibration_counter) / 200.0f;
        if (calibration_counter >= 200) {
            is_calibrating = false;
            cal_feedback.is_calibrating = false;
            cal_feedback.calibration_done = true;
            float avg_ax = sum_ax / 200.0f;
            float avg_ay = sum_ay / 200.0f;
            float avg_az = sum_az / 200.0f;
            float avg_gx = sum_gx / 200.0f;
            float avg_gy = sum_gy / 200.0f;
            float avg_gz = sum_gz / 200.0f;
            float g = 9.80665f;
            float diff_x = std::abs(std::abs(avg_ax) - g);
            float diff_y = std::abs(std::abs(avg_ay) - g);
            float diff_z = std::abs(std::abs(avg_az) - g);
            float bias_ax = avg_ax;
            float bias_ay = avg_ay;
            float bias_az = avg_az;
            float min_diff = diff_x;
            if (diff_y < min_diff) {
                min_diff = diff_y;
            }
            if (diff_z < min_diff) {
                min_diff = diff_z;
            }
            if (min_diff == diff_x) {
                bias_ax = avg_ax > 0.0f ? (avg_ax - g) : (avg_ax + g);
            } else if (min_diff == diff_y) {
                bias_ay = avg_ay > 0.0f ? (avg_ay - g) : (avg_ay + g);
            } else {
                bias_az = avg_az > 0.0f ? (avg_az - g) : (avg_az + g);
            }
            cal_feedback.accel_bias_x = bias_ax;
            cal_feedback.accel_bias_y = bias_ay;
            cal_feedback.accel_bias_z = bias_az;
            cal_feedback.gyro_bias_x = avg_gx;
            cal_feedback.gyro_bias_y = avg_gy;
            cal_feedback.gyro_bias_z = avg_gz;
        }
    }

    if (!allb.cfg_appb.calibrate_requested) {
        cal_feedback.calibration_done = false;
    }

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
 
    accel_raw_mps2 -= cfg_data.navc.accel_bias;
    omega_raw_radps -= cfg_data.navc.gyro_bias;
 
    Eigen::Vector3f accel_body_raw_mps2 = cfg_data.navc.q_IMU2body.conjugate() * accel_raw_mps2;
    Eigen::Vector3f omega_body_radps = cfg_data.navc.q_IMU2body.conjugate() * omega_raw_radps;
 
    IMU_Compensated output;
    output.accel_CG_mps2 = accel_body_raw_mps2;
    output.omega_body_radps = omega_body_radps;
    return output;
}
