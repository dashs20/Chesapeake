#include "alloc.hpp"
#include "../PC.hpp"
#include "../gnc_util/gnc_util.hpp"
#include <cmath>

alloc::alloc(alloc_cfg config)
    : t_min_frac(config.t_min_frac), max_delta_throttle(config.max_delta_throttle),
      gear_ratio(config.gear_ratio),
      servo1_offset_deg(config.servo1_offset_deg),
      servo2_offset_deg(config.servo2_offset_deg) {
  theta_min_rad = gnc_util::deg2rad(config.theta_min_deg);
  theta_max_rad = gnc_util::deg2rad(config.theta_max_deg);
}

alloc::~alloc() {}

// Query member function
act_cmd alloc::query(double alpha_x, double alpha_y, double alpha_z,
                     double t_frac) {

  // clip throttle to prevent divide by zeros
  t_frac = gnc_util::double_clip(t_frac, t_min_frac, 1.0);

  // scale alpha by throttle to get delta command
  double dx = alpha_x / t_frac;
  double dy = alpha_y / t_frac;

  // calculate servo angles
  double arg1 = gnc_util::double_clip((dx - dy) / (2.0 * sqrtf(2)), -1.0, 1.0);
  double arg2 = gnc_util::double_clip((dx + dy) / (2.0 * sqrtf(2)), -1.0, 1.0);
  double theta_1_rad = acos(arg1) - PC::PI / 4.0;
  double theta_2_rad = acos(arg2) - PC::PI / 4.0;

  // saturate theta
  theta_1_rad =
      gnc_util::double_clip(theta_1_rad, theta_min_rad, theta_max_rad);
  theta_2_rad =
      gnc_util::double_clip(theta_2_rad, theta_min_rad, theta_max_rad);

  // compute th and tl
  double alpha_z_sat = gnc_util::double_clip(alpha_z, -max_delta_throttle, max_delta_throttle);
  double th_frac = gnc_util::double_clip(t_frac + alpha_z_sat, t_min_frac, 1.0);
  double tl_frac = gnc_util::double_clip(t_frac - alpha_z_sat, t_min_frac, 1.0);

  act_cmd result;
  result.theta_1_deg =
      ((gnc_util::rad2deg(theta_1_rad) / gear_ratio) + servo1_offset_deg);
  result.theta_2_deg =
      ((gnc_util::rad2deg(theta_2_rad) / gear_ratio) + servo2_offset_deg) * -1 +
      180.0;
  result.th_frac = th_frac;
  result.tl_frac = tl_frac;

  return result;
}
