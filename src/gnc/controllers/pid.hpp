#pragma once

struct pid_cfg {
  double kp;
  double ki;
  double kd;
  double i_lim;
  double looprate_hz;
};

class pid {
public:
  pid(pid_cfg config);                  // Default Constructor
  ~pid();                               // Destructor
  double query(double est, double des); // Query member function

private:
  // gains
  double kp;
  double ki;
  double kd;

  // integral limits
  double i_min;
  double i_max;

  // other
  double i_err;    // current integral of error
  double prev_err; // previous error (for derivative)
  double dt_s;     // dt_s (1/looprate_hz)
};