#pragma once

struct alloc_cfg {
  double t_min_frac;
};

struct act_cmd {
  // Motor throttle fractions (from 0.0 to 1.0)
  double motors[4];
  
  // Servo angles in degrees (from 0.0 to 180.0)
  double servos[4];
};

typedef act_cmd (*allocator_fn)(double alpha_x, double alpha_y, double alpha_z,
                               double t_frac, const alloc_cfg& config);

// Allocator function declarations
act_cmd quad_allocator(double alpha_x, double alpha_y, double alpha_z,
                       double t_frac, const alloc_cfg& config);