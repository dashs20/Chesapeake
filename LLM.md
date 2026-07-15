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

No Commits: Never create a git commit or stage files unless the user explicitly requests it.

Attribution: Include "Assisted by Gemini" in all relevant commit bodies.

Serial Monitor: You are NOT allowed to open the serial monitor anymore. When you want to test, ask the USER and they will do it. Never run pio device monitor or any command to read from the serial port.

Debug Editing: Whenever the debug telemetry (DEBUG.cpp) is edited for a new request, either uncomment an existing block that already matches the request, or comment out the active block and add a new block. Do not delete past telemetry blocks.

Self-Correction: Immediately document any user correction in LLM.md following the established format.

Correction Log:
*   **Correction #1 (2026-07-08)**: Offset Scaling on Servo Commands.
    *   *Mistake*: Assumed GNC servo commands were relative to center and added a 90-degree offset in the HAL layer.
    *   *Advised Solution*: GNC commands servo angles directly, mapping 1:1 to the values written by the HAL module. Never apply offsets to servo commands in the HAL layer.
    *   *Action*: Added a rule preventing offsets in the HAL layer for servo commands, and updated the servo module plan.
*   **Correction #2 (2026-07-09)**: Serial Monitor Lifespan.
    *   *Mistake*: Left serial monitor running in the background, locking up the terminal interface.
    *   *Advised Solution*: Whenever you open the serial monitor, CLOSE IT WHEN YOU'RE DONE. Never leave it running when ending your turn or prompting the user.
    *   *Action*: Added a rule to close the serial monitor immediately after testing and logged the correction.
*   **Correction #3 (2026-07-09)**: Serial Monitor Prohibited.
    *   *Mistake*: Attempted to run the serial monitor, which blocks/interferes with the user's terminal environment.
    *   *Advised Solution*: You are NOT allowed to open the serial monitor anymore. When you want to test, ask the USER and they will do it.
    *   *Action*: Replaced the serial monitor lifespan rule with an absolute prohibition on using the serial monitor, and logged the correction.
*   **Correction #4 (2026-07-14)**: Coordinate Axes Colors and Layout.
    *   *Mistake*: Rendered coordinate axes using non-standard colors (Y as Blue, Z as Green), and positioned the indicator overlapping the main quadcopter graphics.
    *   *Advised Solution*: Standardize 3D coordinate system indicator colors to RGB-to-XYZ mapping (X = Red, Y = Green, Z = Blue). Ensure indicators are compact and sufficiently offset to prevent overlapping any graphic components.
    *   *Action*: Repositioned the indicator to `translate(20, 20)`, shortened the axes to 35px, updated X to Red (`#ef4444`), Y to Green (`#10b981`), Z to Blue (`#3b82f6`), and documented this correction.