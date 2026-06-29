#include "alloc.hpp"
#include "../PC.hpp"
#include "../gnc_util/gnc_util.hpp"
#include <cmath>

alloc::alloc(alloc_cfg config)
    : t_min_frac(config.t_min_frac),
      max_delta_throttle(config.max_delta_throttle),
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

  // calculate servo angles
  double theta_1_rad =
      gnc_util::double_clip(PC::PI / 2 + (alpha_y - alpha_x) / gear_ratio,
                            theta_min_rad, theta_max_rad);
  double theta_2_rad =
      gnc_util::double_clip(PC::PI / 2 + (-alpha_y - alpha_x) / gear_ratio,
                            theta_min_rad, theta_max_rad);

  // compute th and tl
  double alpha_z_sat =
      gnc_util::double_clip(alpha_z, -max_delta_throttle, max_delta_throttle);
  double th_frac = gnc_util::double_clip(t_frac + alpha_z_sat, t_min_frac, 1.0);
  double tl_frac = gnc_util::double_clip(t_frac - alpha_z_sat, t_min_frac, 1.0);

  act_cmd result;
  result.theta_1_deg = gnc_util::rad2deg(theta_1_rad) + servo1_offset_deg;
  result.theta_2_deg = gnc_util::rad2deg(theta_2_rad) + servo2_offset_deg;
  result.th_frac = th_frac;
  result.tl_frac = tl_frac;

  return result;
}
