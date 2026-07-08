#include "alloc.hpp"
#include "../gnc_util/gnc_util.hpp"

act_cmd quad_allocator(double alpha_x, double alpha_y, double alpha_z,
                       double t_frac, const alloc_cfg& config) {
  // clip throttle to prevent divide by zeros
  t_frac = gnc_util::double_clip(t_frac, config.t_min_frac, 1.0);

  // Betaflight QuadX mixing:
  // Roll = alpha_x, Pitch = alpha_y, Yaw = alpha_z
  // Motor 1 (Back Right) = Throttle - Roll + Pitch + Yaw
  double m1 = t_frac - alpha_x + alpha_y + alpha_z;
  
  // Motor 2 (Front/Top Right) = Throttle - Roll - Pitch - Yaw
  double m2 = t_frac - alpha_x - alpha_y - alpha_z;
  
  // Motor 3 (Back Left) = Throttle + Roll + Pitch - Yaw
  double m3 = t_frac + alpha_x + alpha_y - alpha_z;
  
  // Motor 4 (Front Left) = Throttle + Roll - Pitch + Yaw
  double m4 = t_frac + alpha_x - alpha_y + alpha_z;

  act_cmd result = {};
  result.servos[0] = 0.0;
  result.servos[1] = 0.0;
  result.servos[2] = 0.0;
  result.servos[3] = 0.0;

  result.motors[0] = gnc_util::double_clip(m1, config.t_min_frac, 1.0);
  result.motors[1] = gnc_util::double_clip(m2, config.t_min_frac, 1.0);
  result.motors[2] = gnc_util::double_clip(m3, config.t_min_frac, 1.0);
  result.motors[3] = gnc_util::double_clip(m4, config.t_min_frac, 1.0);

  return result;
}
