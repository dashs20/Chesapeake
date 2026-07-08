#pragma once
#include "gnc/gnc.hpp"

struct gnc_params_t {
  double looprate_hz;
  double max_rate_degps;
  double allocator_type;
  gnc_cfg spacey_config;
};

extern gnc_params_t gnc_params;

// Backward compatibility with legacy GNC variables
extern double looprate_hz;
extern double max_rate_degps;
extern gnc_cfg spacey_config;

void gnc_config_load_defaults();
void gnc_config_sync_legacy();
bool gnc_config_set(const char* name, double value);
bool gnc_config_get(const char* name, double& value);
void gnc_config_print_all();