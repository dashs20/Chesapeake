#pragma once
#include "allocation/alloc.hpp"

// Generic Inputs structure for the State Machine
struct VsmInput {
  bool is_link_up;
  double arm_channel_val;
};

// State ID Enumeration
enum class VsmStateId {
  DISARMED = 0,
  PREFLIGHT = 1,
  ARMED = 2
};

class Vsm; // Forward declaration

// Generic State Base Class
class VsmState {
public:
  VsmState(VsmStateId id, Vsm* machine) : id(id), machine(machine) {}
  virtual ~VsmState() = default;

  VsmStateId get_id() const { return id; }

  virtual void on_entry() {}
  virtual void on_exit() {}

  // Transition check: returns pointer to next state on transition, or nullptr/this to stay
  virtual VsmState* update(const VsmInput& input) = 0;

  // Enforces state-specific safety constraints on nominal actuator commands
  virtual act_cmd filter_commands(const act_cmd& nominal_cmd) const {
    return nominal_cmd;
  }

protected:
  VsmStateId id;
  Vsm* machine;
};

// Generic State Machine Base Class
class Vsm {
public:
  Vsm() : current_state(nullptr) {}
  virtual ~Vsm() = default;

  void init(VsmState* initial_state) {
    current_state = initial_state;
    if (current_state) {
      current_state->on_entry();
    }
  }

  void update(const VsmInput& input) {
    if (!current_state) return;
    
    VsmState* next_state = current_state->update(input);
    if (next_state && next_state != current_state) {
      current_state->on_exit();
      current_state = next_state;
      current_state->on_entry();
    }
  }

  act_cmd filter_commands(const act_cmd& nominal_cmd) const {
    return current_state ? current_state->filter_commands(nominal_cmd) : nominal_cmd;
  }

  VsmStateId get_state_id() const {
    return current_state ? current_state->get_id() : VsmStateId::DISARMED;
  }

protected:
  VsmState* current_state;
};

// Concrete Flight State Implementations

class DisarmedState : public VsmState {
public:
  DisarmedState(Vsm* machine, VsmState* preflight, VsmState* armed) 
      : VsmState(VsmStateId::DISARMED, machine), preflight_state(preflight), armed_state(armed) {}
  
  void on_entry() override;
  void on_exit() override;
  VsmState* update(const VsmInput& input) override;
  act_cmd filter_commands(const act_cmd& nominal_cmd) const override;

private:
  VsmState* preflight_state;
  VsmState* armed_state;
};

class PreflightState : public VsmState {
public:
  PreflightState(Vsm* machine, VsmState* disarmed, VsmState* armed) 
      : VsmState(VsmStateId::PREFLIGHT, machine), disarmed_state(disarmed), armed_state(armed) {}
  
  void on_entry() override;
  void on_exit() override;
  VsmState* update(const VsmInput& input) override;
  act_cmd filter_commands(const act_cmd& nominal_cmd) const override;

private:
  VsmState* disarmed_state;
  VsmState* armed_state;
};

class ArmedState : public VsmState {
public:
  ArmedState(Vsm* machine, VsmState* disarmed, VsmState* preflight) 
      : VsmState(VsmStateId::ARMED, machine), disarmed_state(disarmed), preflight_state(preflight) {}
  
  void on_entry() override;
  void on_exit() override;
  VsmState* update(const VsmInput& input) override;
  act_cmd filter_commands(const act_cmd& nominal_cmd) const override;

private:
  VsmState* disarmed_state;
  VsmState* preflight_state;
};

// Concrete Flight State Machine
class FlightVsm : public Vsm {
public:
  FlightVsm();
  void begin();
  const char* get_state_string() const;

private:
  DisarmedState disarmed_state;
  PreflightState preflight_state;
  ArmedState armed_state;
};
