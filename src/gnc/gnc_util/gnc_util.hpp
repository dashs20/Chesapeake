#pragma once
#include <map>
#include <string>

namespace gnc_util {
double double_clip(double x, double x_min, double x_max); // clip/saturate
double hz2ps(double hz);                                  // Hz to per second
double deg2rad(double deg);                               // degrees to radians
double rad2deg(double rad);
struct vec { // vector struct (we don't use a ton of vectors in here, so just
             // declaring this instead of using Eigen)
  double x;
  double y;
  double z;
};
vec euler_xyz_rotate_deg(vec input_vec,
                         vec angles_deg); // euler rotater (for IMU alignment)

} // namespace gnc_util
