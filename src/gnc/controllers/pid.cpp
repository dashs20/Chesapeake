#include "pid.hpp"
#include "../gnc_util/gnc_util.hpp"
#include <cmath>

// constructor
pid::pid(pid_cfg config) : kp(config.kp), ki(config.ki), kd(config.kd) {

  // Initialize
  i_min = -std::abs(config.i_lim);
  i_max = std::abs(config.i_lim);
  i_err = 0;
  prev_err = 0;
  dt_s = 1.0 / config.looprate_hz;
}

// filter method
double pid::query(double est, double des) {
  // compute error
  double err = des - est;

  // update integral (trapezoidal)
  double mean_err = (prev_err + err) / 2;
  i_err += mean_err * dt_s;
  i_err = gnc_util::double_clip(i_err, i_min, i_max); // clip integral

  // compute derivative (euler)
  double d_err = (err - prev_err) / dt_s;

  // update prev_err
  prev_err = err;

  // compute control signal
  return kp * err + ki * i_err + kd * d_err;
}

// destructor
pid::~pid() {}