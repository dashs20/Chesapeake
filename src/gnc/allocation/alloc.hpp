#pragma once

struct alloc_cfg {
  double t_min_frac;
  double theta_min_deg;
  double theta_max_deg;
  double gear_ratio;
  double servo1_offset_deg;
  double servo2_offset_deg;
};

struct act_cmd {
  double theta_1_deg;
  double theta_2_deg;
  double th_frac;
  double tl_frac;
};

class alloc {
public:
  alloc(alloc_cfg config); // Default Constructor
  ~alloc();                // Destructor
  act_cmd query(double alpha_x, double alpha_y, double alpha_z,
                double t_frac); // Query member function

private:
  // throttle limits
  double t_min_frac;

  // theta limits
  double theta_min_rad;
  double theta_max_rad;

  // servo gearing
  double gear_ratio;

  // servo offsets
  double servo1_offset_deg;
  double servo2_offset_deg;
};