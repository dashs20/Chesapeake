#include "alloc.hpp"
#include "../PC.hpp"
#include "../util/util.hpp"
#include <cmath>

alloc::alloc(alloc_cfg config) : t_min_frac(config.t_min_frac) {
  theta_min_rad = util::deg2rad(config.theta_min_deg);
  theta_max_rad = util::deg2rad(config.theta_max_deg);
}

alloc::~alloc() {}

// Query member function
act_cmd alloc::query(double alpha_x, double alpha_y, double alpha_z,
                     double t_frac) {

  // scale alpha by throttle to get delta command
  double dx = alpha_x / t_frac;
  double dy = alpha_y / t_frac;

  // calculate servo angles
  double theta_1_rad = acos((dx - dy) / 2.0 * sqrtf(2)) - PC::PI / 4.0;
  double theta_2_rad = acos((dx + dy) / 2.0 * sqrtf(2)) - PC::PI / 4.0;

  // saturate theta
  theta_1_rad = util::double_clip(theta_1_rad, theta_min_rad, theta_max_rad);
  theta_2_rad = util::double_clip(theta_2_rad, theta_min_rad, theta_max_rad);

  // compute th and tl
  double th_frac = util::double_clip(t_frac + alpha_z, t_min_frac, 1.0);
  double tl_frac = util::double_clip(t_frac - alpha_z, t_min_frac, 1.0);

  act_cmd result;
  result.theta_1_rad = theta_1_rad;
  result.theta_2_rad = theta_2_rad;
  result.th_frac = th_frac;
  result.tl_frac = tl_frac;

  return result;
}
