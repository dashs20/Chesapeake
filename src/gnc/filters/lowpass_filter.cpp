#include "lowpass_filter.hpp"
#include "../PC.hpp"
#include "../gnc_util/gnc_util.hpp"

// Default constructor
lpf::lpf(lpf_cfg config) {
  prev_output = 0;
  double dt_s = gnc_util::hz2ps(config.looprate_hz);
  alpha = (2 * PC::PI * config.fc_hz * dt_s) /
          (2 * PC::PI * config.fc_hz * dt_s + 1);
}

// filter method
double lpf::filter(double input) {
  double output = alpha * input + (1 - alpha) * prev_output;
  prev_output = output;
  return output;
}

// Destructor
lpf::~lpf() {}