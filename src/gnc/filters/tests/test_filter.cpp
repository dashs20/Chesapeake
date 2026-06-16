#include "../../PC.hpp"
#include "../lowpass_filter.hpp"
#include <cmath>
#include <iostream>
#include <random>

int main() {
  // build the filter
  double looprate_hz = 1000.0;
  double dt_s = 1.0 / looprate_hz;
  double t_end_s = 2;

  int n_steps = round(t_end_s * looprate_hz);

  lpf_cfg config = {5.0, looprate_hz};
  lpf lowpass(config);

  // define the noisy input
  double sine_freq_hz = 1;
  double sine_freq_radps = sine_freq_hz * 2 * PC::PI;
  double time_s[n_steps];
  double raw_output[n_steps];
  double filtered_output[n_steps];
  double cur_time = 0;

  std::default_random_engine gen;
  std::uniform_real_distribution<double> unif(-0.4, 0.4);

  for (int i = 0; i < n_steps; i++) {
    time_s[i] = cur_time;
    double noise = unif(gen);
    raw_output[i] = std::sin(time_s[i] * sine_freq_radps) + noise;
    cur_time += dt_s;
  }

  // filter the input
  std::cout << "time_s,raw_output,filtered_output" << std::endl;
  for (int i = 0; i < n_steps; i++) {
    filtered_output[i] = lowpass.filter(raw_output[i]);
    std::cout << time_s[i] << "," << raw_output[i] << "," << filtered_output[i]
              << std::endl;
  }

  return 0;
}