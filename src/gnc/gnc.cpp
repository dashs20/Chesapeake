#include "gnc.hpp"
#include "allocation/alloc.hpp"
#include "controllers/pid.hpp"
#include "filters/lowpass_filter.hpp"
#include "gnc_util/gnc_util.hpp"

gnc::gnc(gnc_cfg config)
    : config(config), imu_x_lpf(config.imu_lpf_cfg),
      imu_y_lpf(config.imu_lpf_cfg), imu_z_lpf(config.imu_lpf_cfg),
      pid_x(config.pid_x_cfg), pid_y(config.pid_y_cfg), pid_z(config.pid_z_cfg) {
  if (config.allocator_type == AllocatorType::VTVL) {
    allocator_func = VTVL_allocator;
  } else {
    allocator_func = quad_allocator;
  }
}

gnc::~gnc() {}

act_cmd gnc::query(gnc_util::vec imu_raw_degps, gnc_util::vec rate_cmd_degps,
                   double thr_frac) {

  // Rotate IMU signal
  gnc_util::vec imu_raw_rotated_degps =
      gnc_util::euler_xyz_rotate_deg(imu_raw_degps, config.imu_euler_xyz_deg);

  // filter IMU signal
  double imu_filt_x_degps = imu_x_lpf.filter(imu_raw_rotated_degps.x);
  double imu_filt_y_degps = imu_y_lpf.filter(imu_raw_rotated_degps.y);
  double imu_filt_z_degps = imu_z_lpf.filter(imu_raw_rotated_degps.z);

  // PID control
  double alpha_x = pid_x.query(imu_filt_x_degps, rate_cmd_degps.x);
  double alpha_y = pid_y.query(-imu_filt_y_degps, rate_cmd_degps.y);
  double alpha_z = pid_z.query(imu_filt_z_degps, rate_cmd_degps.z);

  // Allocation
  return allocator_func(alpha_x, alpha_y, alpha_z, thr_frac, config.veh_alloc_cfg);
}
