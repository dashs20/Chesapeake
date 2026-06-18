#include "gnc_util.hpp"
#include "../PC.hpp"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>

namespace gnc_util {

double double_clip(double x, double x_min, double x_max) {
  if (x > x_max) {
    x = x_max;
  }
  if (x < x_min) {
    x = x_min;
  }
  return x;
}

double hz2ps(double hz) { return 1.0 / hz; }

double deg2rad(double deg) { return PC::PI / 180.0 * deg; }
double rad2deg(double rad) { return 180.0 / PC::PI * rad; }

vec euler_xyz_rotate_deg(vec input_vec, vec angles_deg) {

  // 0. Pre-compute trig and convert to rad
  double cx = std::cos(gnc_util::deg2rad(angles_deg.x));
  double sx = std::sin(gnc_util::deg2rad(angles_deg.x));
  double cy = std::cos(gnc_util::deg2rad(angles_deg.y));
  double sy = std::sin(gnc_util::deg2rad(angles_deg.y));
  double cz = std::cos(gnc_util::deg2rad(angles_deg.z));
  double sz = std::sin(gnc_util::deg2rad(angles_deg.z));

  // 1. Rotate around X-axis
  double x1 = input_vec.x;
  double y1 = input_vec.y * cx - input_vec.z * sx;
  double z1 = input_vec.y * sx + input_vec.z * cx;

  // 2. Rotate around Y-axis
  double x2 = x1 * cy + z1 * sy;
  double y2 = y1;
  double z2 = -x1 * sy + z1 * cy;

  // 3. Rotate around Z-axis
  vec result;
  result.x = x2 * cz - y2 * sz;
  result.y = x2 * sz + y2 * cz;
  result.z = z2;
  return result;
}

} // namespace gnc_util
