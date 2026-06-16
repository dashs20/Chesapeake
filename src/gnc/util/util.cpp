#include "util.hpp"
#include "../PC.hpp"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>

namespace util {

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

vec euler_xyz_rotate_deg(vec input_vec, vec angles_deg) {

  // 0. Pre-compute trig and convert to rad
  double cx = std::cos(util::deg2rad(angles_deg.x));
  double sx = std::sin(util::deg2rad(angles_deg.x));
  double cy = std::cos(util::deg2rad(angles_deg.y));
  double sy = std::sin(util::deg2rad(angles_deg.y));
  double cz = std::cos(util::deg2rad(angles_deg.z));
  double sz = std::sin(util::deg2rad(angles_deg.z));

  // 1. Rotate around local Z-axis
  double x1 = input_vec.x * cz - input_vec.y * sz;
  double y1 = input_vec.x * sz + input_vec.y * cz;
  double z1 = input_vec.z;

  // 2. Rotate around local Y-axis
  double x2 = x1 * cy + z1 * sy;
  double y2 = y1;
  double z2 = -x1 * sy + z1 * cy;

  // 3. Rotate around local X-axis
  vec result;
  result.x = x2;
  result.y = y2 * cx - z2 * sx;
  result.z = y2 * sx + z2 * cx;
  return result;
}

// Config Parser Implementation
config_parser::config_parser() {}
config_parser::~config_parser() {}

void config_parser::trim(std::string &s) {
  s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](unsigned char ch) {
            return !std::isspace(ch);
          }));
  s.erase(std::find_if(s.rbegin(), s.rend(),
                       [](unsigned char ch) { return !std::isspace(ch); })
              .base(),
          s.end());
}

bool config_parser::load_file(const std::string &filepath) {
  std::ifstream file(filepath);
  if (!file.is_open()) {
    std::cerr << "Failed to open config file: " << filepath << std::endl;
    return false;
  }

  std::string line;
  while (std::getline(file, line)) {
    trim(line);

    // Skip empty lines and comments
    if (line.empty() || line[0] == '#') {
      continue;
    }

    // Look for "set key = value"
    if (line.substr(0, 4) == "set ") {
      std::string assignment = line.substr(4);
      size_t eq_pos = assignment.find('=');
      if (eq_pos != std::string::npos) {
        std::string key = assignment.substr(0, eq_pos);
        std::string val_str = assignment.substr(eq_pos + 1);

        trim(key);
        trim(val_str);

        try {
          double value = std::stod(val_str);
          parameters[key] = value;
        } catch (...) {
          std::cerr << "Warning: Could not parse value for key '" << key
                    << "' in line: " << line << std::endl;
        }
      }
    }
  }

  return true;
}

double config_parser::get(const std::string &key) {
  if (parameters.find(key) != parameters.end()) {
    return parameters[key];
  }
  throw std::runtime_error("CRITICAL: Configuration key '" + key +
                           "' missing from file!");
}

} // namespace util
