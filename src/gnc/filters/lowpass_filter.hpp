#pragma once

struct lpf_cfg {
  double fc_hz;
  double looprate_hz;
};

class lpf {
public:
  lpf(lpf_cfg config);         // Default Constructor
  ~lpf();                      // Destructor
  double filter(double input); // Filter member function

private:
  // private, internal variables
  double prev_output;
  double alpha;
};