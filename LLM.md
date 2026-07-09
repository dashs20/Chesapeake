Architecture & Data Flow
Enforce Separation: Maintain strict boundaries between HAL, CONFIGURATOR, and GNC.

Use Read-Only Master Buses: Pass const GNCb& (or HALb) into submodule update() methods.

Return Only Sub-buses: Submodules must return only their designated sub-bus or field (e.g., NAVb, VSMb), leaving the coordinator to update the master bus.

No Redundant State: Never store bus structures (e.g., NAVb, CTLb) as class member variables.

Centralize Configurations: Consolidate all configs into src/GNC/cfg.hpp under the master GNCc struct.

Nest Configurations: Nest component configs within their parent module configs (e.g., nest PID_3DOFc inside CTLc, not globally).

Servo Mapping: GNC commands servo angles directly. They map 1:1 to what the HAL module writes. Never apply offsets (like adding 90 degrees) to servo commands in the HAL layer.

Naming Conventions
Bus Structs: ALL CAPS + b suffix (e.g., GNCb, IMUb, RCRXb).

Config Structs: ALL CAPS + c suffix (e.g., NAVc, PID_SCALARc, PID_3DOFc).

Instance Names: Exact lowercase match of type name (e.g., navb for NAVb), unless multiple instances exist.

No Redundant Bus Suffixes: Omit context suffixes (_des, _est, _cmd) inside bus definitions; the bus type defines the context.

Mandatory Unit Suffixes: Append lowercase physical units to all applicable variables (e.g., _s, _radps, _mps2, _frac). No exceptions.

Quaternion Naming: Strictly use q_<FRAMEA>2<FRAMEB>.

Self-Documenting Variables: Use full, descriptive variable names. Forbid short/cryptic locals (T, R, def_s).

Types & Math
Enforce Single-Precision: Use float, Eigen::Vector3f, and Eigen::Quaternionf. Explicitly cast doubles via .cast<float>().

Standard Math: Use standard library <cmath> functions (e.g., std::pow()), never manual geometric expansion.

Enums: Always use scoped enum class.

Control Flow
Binary/Unary Checks: Use if-else or ternaries for states with 1 or 2 options.

Complex States: Use switch exclusively for enums with 3 or more defined cases.

Styling & Workflow
Zero Source Comments: Forbid comments in standard .cpp and .hpp files. (Exceptions: cfg.hpp, bus.hpp). Rely on clean vertical whitespace.

No Global Namespaces: Use fully qualified paths (e.g., Eigen::Vector3f).

Required Directives: Begin every header with #pragma once. Ensure all structs end with ;.

Compilation: Never invoke pio run unless explicitly requested or a valid main.cpp entrypoint is verified.

Attribution: Include "Assisted by Gemini" in all relevant commit bodies.

Self-Correction: Immediately document any user correction in LLM.md following the established format.

Correction Log:
*   **Correction #1 (2026-07-08)**: Offset Scaling on Servo Commands.
    *   *Mistake*: Assumed GNC servo commands were relative to center and added a 90-degree offset in the HAL layer.
    *   *Advised Solution*: GNC commands servo angles directly, mapping 1:1 to the values written by the HAL module. Never apply offsets to servo commands in the HAL layer.
    *   *Action*: Added a rule preventing offsets in the HAL layer for servo commands, and updated the servo module plan.