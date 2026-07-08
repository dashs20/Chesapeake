#include "alloc.hpp"
#include "../PC.hpp"
#include "../gnc_util/gnc_util.hpp"
#include <cmath>

act_cmd VTVL_allocator(double alpha_x, double alpha_y, double alpha_z,
                       double t_frac, const alloc_cfg& config) {
  // clip throttle to prevent divide by zeros
  t_frac = gnc_util::double_clip(t_frac, config.t_min_frac, 1.0);

  double theta_min_rad = gnc_util::deg2rad(config.theta_min_deg);
  double theta_max_rad = gnc_util::deg2rad(config.theta_max_deg);

  // calculate servo angles
  double theta_1_rad =
      gnc_util::double_clip(PC::PI / 2 + (alpha_y - alpha_x) / config.gear_ratio,
                            theta_min_rad, theta_max_rad);
  double theta_2_rad =
      gnc_util::double_clip(PC::PI / 2 + (-alpha_y - alpha_x) / config.gear_ratio,
                            theta_min_rad, theta_max_rad);

  // compute th and tl
  double alpha_z_sat =
      gnc_util::double_clip(alpha_z, -config.max_delta_throttle, config.max_delta_throttle);
  double th_frac = gnc_util::double_clip(t_frac - alpha_z_sat, config.t_min_frac, 1.0);
  double tl_frac = gnc_util::double_clip(t_frac + alpha_z_sat, config.t_min_frac, 1.0);

  act_cmd result = {};
  result.servos[0] = gnc_util::rad2deg(theta_1_rad) + config.servo1_offset_deg;
  result.servos[1] = gnc_util::rad2deg(theta_2_rad) + config.servo2_offset_deg;
  result.servos[2] = 0.0;
  result.servos[3] = 0.0;
  
  result.motors[0] = th_frac;
  result.motors[1] = tl_frac;
  result.motors[2] = 0.0;
  result.motors[3] = 0.0;

  return result;
}
