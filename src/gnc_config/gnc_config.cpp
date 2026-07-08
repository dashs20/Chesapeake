#include "gnc_config.hpp"
#include <string.h>
#include <Arduino.h>

gnc_params_t gnc_params;

// Define legacy global variables
double looprate_hz = 1000.0;
double max_rate_degps = 100.0;
double battery_multiplier = 10.1;
gnc_cfg chesapeake_config;

struct DoubleParam {
  const char* name;
  double* value_ptr;
};

// Map parameter names to struct members
static DoubleParam gnc_double_params[] = {
  {"looprate_hz", &gnc_params.looprate_hz},
  {"max_rate_degps", &gnc_params.max_rate_degps},
  {"battery_multiplier", &gnc_params.battery_multiplier},
  {"pid_x_kp", &gnc_params.chesapeake_config.pid_x_cfg.kp},
  {"pid_x_ki", &gnc_params.chesapeake_config.pid_x_cfg.ki},
  {"pid_x_kd", &gnc_params.chesapeake_config.pid_x_cfg.kd},
  {"pid_x_ilim", &gnc_params.chesapeake_config.pid_x_cfg.i_lim},
  {"pid_y_kp", &gnc_params.chesapeake_config.pid_y_cfg.kp},
  {"pid_y_ki", &gnc_params.chesapeake_config.pid_y_cfg.ki},
  {"pid_y_kd", &gnc_params.chesapeake_config.pid_y_cfg.kd},
  {"pid_y_ilim", &gnc_params.chesapeake_config.pid_y_cfg.i_lim},
  {"pid_z_kp", &gnc_params.chesapeake_config.pid_z_cfg.kp},
  {"pid_z_ki", &gnc_params.chesapeake_config.pid_z_cfg.ki},
  {"pid_z_kd", &gnc_params.chesapeake_config.pid_z_cfg.kd},
  {"pid_z_ilim", &gnc_params.chesapeake_config.pid_z_cfg.i_lim},
  {"t_min_frac", &gnc_params.chesapeake_config.veh_alloc_cfg.t_min_frac},
  {"imu_lpf_fc_hz", &gnc_params.chesapeake_config.imu_lpf_cfg.fc_hz},
  {"imu_euler_x", &gnc_params.chesapeake_config.imu_euler_xyz_deg.x},
  {"imu_euler_y", &gnc_params.chesapeake_config.imu_euler_xyz_deg.y},
  {"imu_euler_z", &gnc_params.chesapeake_config.imu_euler_xyz_deg.z}
};

static const size_t num_gnc_double_params = sizeof(gnc_double_params) / sizeof(DoubleParam);

void gnc_config_load_defaults() {
  gnc_params.looprate_hz = 1000.0;
  gnc_params.max_rate_degps = 100.0;
  gnc_params.battery_multiplier = 10.1;
  
  gnc_params.chesapeake_config.pid_x_cfg = {.kp = 0.0022, .ki = 0.0, .kd = 0.0, .i_lim = 20.0, .looprate_hz = 1000.0};
  gnc_params.chesapeake_config.pid_y_cfg = {.kp = 0.0022, .ki = 0.0, .kd = 0.0, .i_lim = 20.0, .looprate_hz = 1000.0};
  gnc_params.chesapeake_config.pid_z_cfg = {.kp = 0.0001, .ki = 0.0, .kd = 0.0, .i_lim = 20.0, .looprate_hz = 1000.0};
  
  gnc_params.chesapeake_config.veh_alloc_cfg = {
    .t_min_frac = 0.05
  };
  
  gnc_params.chesapeake_config.imu_lpf_cfg = {.fc_hz = 50.0, .looprate_hz = 1000.0};
  gnc_params.chesapeake_config.imu_euler_xyz_deg = {.x = 0.0, .y = 0.0, .z = 0.0};
  
  gnc_config_sync_legacy();
}

void gnc_config_sync_legacy() {
  looprate_hz = gnc_params.looprate_hz;
  max_rate_degps = gnc_params.max_rate_degps;
  battery_multiplier = gnc_params.battery_multiplier;
  
  // Update the nested looprate_hz variables inside components config
  gnc_params.chesapeake_config.pid_x_cfg.looprate_hz = gnc_params.looprate_hz;
  gnc_params.chesapeake_config.pid_y_cfg.looprate_hz = gnc_params.looprate_hz;
  gnc_params.chesapeake_config.pid_z_cfg.looprate_hz = gnc_params.looprate_hz;
  gnc_params.chesapeake_config.imu_lpf_cfg.looprate_hz = gnc_params.looprate_hz;
  
  chesapeake_config = gnc_params.chesapeake_config;
}

bool gnc_config_set(const char* name, double value) {
  for (size_t i = 0; i < num_gnc_double_params; ++i) {
    if (strcasecmp(name, gnc_double_params[i].name) == 0) {
      *(gnc_double_params[i].value_ptr) = value;
      gnc_config_sync_legacy();
      return true;
    }
  }
  return false;
}

bool gnc_config_get(const char* name, double& value) {
  for (size_t i = 0; i < num_gnc_double_params; ++i) {
    if (strcasecmp(name, gnc_double_params[i].name) == 0) {
      value = *(gnc_double_params[i].value_ptr);
      return true;
    }
  }
  return false;
}

void gnc_config_print_all() {
  for (size_t i = 0; i < num_gnc_double_params; ++i) {
    Serial.print("set ");
    Serial.print(gnc_double_params[i].name);
    Serial.print(" = ");
    Serial.println(*(gnc_double_params[i].value_ptr), 6);
  }
}
