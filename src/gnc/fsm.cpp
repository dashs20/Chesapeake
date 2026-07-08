#include "fsm.hpp"
#include <Arduino.h>

// ==========================================
// DisarmedState Implementation
// ==========================================

void DisarmedState::on_entry() {
  Serial.println("VSM State: DISARMED");
}

void DisarmedState::on_exit() {
  // Logic upon leaving DISARMED (none needed for now)
}

VsmState* DisarmedState::update(const VsmInput& input) {
  if (!input.is_link_up) {
    return this; // Stay disarmed if link is down
  }

  if (input.arm_channel_val > 1400 && input.arm_channel_val < 1600) {
    return preflight_state;
  }
  if (input.arm_channel_val > 1800) {
    return armed_state;
  }
  return this;
}

act_cmd DisarmedState::filter_commands(const act_cmd& nominal_cmd) const {
  act_cmd safe_cmd = nominal_cmd;
  for (int i = 0; i < 4; i++) {
    safe_cmd.motors[i] = 0.0;
    safe_cmd.servos[i] = 90.0; // Centered neutral
  }
  return safe_cmd;
}

// ==========================================
// PreflightState Implementation
// ==========================================

void PreflightState::on_entry() {
  Serial.println("VSM State: PREFLIGHT");
}

void PreflightState::on_exit() {
  // Logic upon leaving PREFLIGHT
}

VsmState* PreflightState::update(const VsmInput& input) {
  if (!input.is_link_up || (input.arm_channel_val > 900 && input.arm_channel_val < 1200)) {
    return disarmed_state;
  }
  if (input.arm_channel_val > 1800) {
    return armed_state;
  }
  return this;
}

act_cmd PreflightState::filter_commands(const act_cmd& nominal_cmd) const {
  act_cmd safe_cmd = nominal_cmd;
  for (int i = 0; i < 4; i++) {
    safe_cmd.motors[i] = 0.0; // Keep motors off in PREFLIGHT
  }
  // Servos are allowed to pass through (deflection verification)
  return safe_cmd;
}

// ==========================================
// ArmedState Implementation
// ==========================================

void ArmedState::on_entry() {
  Serial.println("VSM State: ARMED");
}

void ArmedState::on_exit() {
  Serial.println("VSM State: DISARMING FROM FLIGHT");
}

VsmState* ArmedState::update(const VsmInput& input) {
  if (!input.is_link_up || (input.arm_channel_val > 900 && input.arm_channel_val < 1200)) {
    return disarmed_state;
  }
  if (input.arm_channel_val > 1400 && input.arm_channel_val < 1600) {
    return preflight_state;
  }
  return this;
}

act_cmd ArmedState::filter_commands(const act_cmd& nominal_cmd) const {
  // Full control pass-through when armed
  return nominal_cmd;
}

// ==========================================
// FlightVsm Implementation
// ==========================================

FlightVsm::FlightVsm() 
    : disarmed_state(this, &preflight_state, &armed_state),
      preflight_state(this, &disarmed_state, &armed_state),
      armed_state(this, &disarmed_state, &preflight_state) {}

void FlightVsm::begin() {
  init(&disarmed_state);
}

const char* FlightVsm::get_state_string() const {
  if (!current_state) return "UNKNOWN";
  switch (current_state->get_id()) {
    case VsmStateId::DISARMED:  return "DISARMED";
    case VsmStateId::PREFLIGHT: return "PREFLIGHT";
    case VsmStateId::ARMED:     return "ARMED";
    default:                    return "UNKNOWN";
  }
}
