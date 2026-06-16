#pragma once
#include <map>
#include <string>

namespace util {
double double_clip(double x, double x_min, double x_max); // clip/saturate
double hz2ps(double hz);                                  // Hz to per second
double deg2rad(double deg);                               // degrees to radians
struct vec { // vector struct (we don't use a ton of vectors in here, so just
             // declaring this instead of using Eigen)
  double x;
  double y;
  double z;
};
vec euler_xyz_rotate_deg(vec input_vec,
                         vec angles_deg); // euler rotater (for IMU alignment)

class config_parser {
public:
  config_parser();
  ~config_parser();

  bool load_file(const std::string &filepath);
  double get(const std::string &key);

private:
  std::map<std::string, double> parameters;
  void trim(std::string &s);
};

} // namespace util
