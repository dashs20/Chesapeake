#pragma once

struct alloc_cfg {
  double t_min_frac;
  double max_delta_throttle;
  double theta_min_deg;
  double theta_max_deg;
  double gear_ratio;
  double servo1_offset_deg;
  double servo2_offset_deg;
};

struct act_cmd {
  // Motor throttle fractions (from 0.0 to 1.0)
  double motors[4];
  
  // Servo angles in degrees (from 0.0 to 180.0)
  double servos[4];
};

enum class AllocatorType {
  VTVL = 0,
  QUAD = 1
};

typedef act_cmd (*allocator_fn)(double alpha_x, double alpha_y, double alpha_z,
                               double t_frac, const alloc_cfg& config);

// Allocator function declarations
act_cmd VTVL_allocator(double alpha_x, double alpha_y, double alpha_z,
                       double t_frac, const alloc_cfg& config);

act_cmd quad_allocator(double alpha_x, double alpha_y, double alpha_z,
                       double t_frac, const alloc_cfg& config);