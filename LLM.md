# Chesapeake Codebase Standards & Conventions

This document outlines the architectural patterns, directory structure, and coding standards for the Chesapeake PlatformIO project. Any LLM working on this codebase should adhere to these guidelines.

---

## 1. Project Directory Structure

The project follows a modular structure where hardware abstraction, core algorithms, and state machines are cleanly separated.

*   **`src/`**: Main application source files.
    *   **`HAL/`**: Hardware Abstraction Layer. Contains code interacting directly with sensors and peripherals (currently empty).
    *   **`CONFIGURATOR/`**: Tooling and assets related to ground control / configurator (currently empty).
    *   **`GNC/`**: Guidance, Navigation, and Control.
        *   **`bus.hpp`**: Define central data structures (buses) passing information between modules.
        *   **`NAV/`**: State estimation/navigation algorithms (e.g., UKF-based navigation).
        *   **`CTL/`**: Flight control algorithms.
            *   **`PID/`**: Contains the local PID controller class.
*   **`lib/`**: External libraries and hardware drivers (e.g., `eigen-master`, `UKF-main`).

---

## 2. Bus Architecture (`bus.hpp`)

All communication between high-level blocks is handled via defined structures in [bus.hpp](file:///src/GNC/bus.hpp).
*   **Master Bus (`GNCb`)**: Every primary GNC submodule (like `NAV` and `CTL`) shares the exact same method interface: they take in a `GNCb` master bus object, perform their respective computations, update their specific segment of the bus, and return the updated `GNCb` object.
    ```cpp
    GNCb update(GNCb gnc);
    ```
*   **Naming Convention**: Inter-module structs use a suffix `b` (e.g., `HALb` for HAL bus, `NAVb` for NAV bus, `CTLb` for Control bus).
*   **Instance Naming Rule**: Instances of bus and configuration structs must have names that are identical to their type names, except for capitalization (e.g., `NAVb` type has instance name `navb`, `HALb` type has instance name `halb`), **UNLESS** there are multiple instances of the same type within a parent struct (e.g., `PIDc` having multiple instances like `rate` and `angle`).
*   **No Redundant Class State**: Modules must **not** store redundant bus structures (like `NAVb` or `CTLb`) as class member variables. Since submodules receive the master bus `GNCb` as an input to their `update()` method and return the updated bus, storing duplicate copies of these bus structures within the class creates redundant state, memory overhead, and risk of desynchronization. Output values must be written directly to the input `GNCb` bus structure.
*   **Enums**: Status and state enums are defined as scoped `enum class` to prevent name clashes (e.g., `enum class STATE` and `enum class CONTROL_MODE`).
*   **Data Types**: Physical 3D vectors use `Eigen::Vector3f` and orientations use `Eigen::Quaternionf` to ensure compatibility with single-precision hardware FPUs.
*   **Bus/Config Struct Naming**: All bus and configuration structure type names must be written in ALL CAPS except for the final suffix character `b` or `c` (e.g. `GNCb`, `NAVc`, `PID_SCALARc`).
*   **No Redundant Suffixes in Buses**: Do not append suffixes like `_des`, `_cmd`, `_est`, or `_meas` to variables if their context is already clear from the bus they reside in. For example, in the Guidance bus `GUIb`, the desired attitude is simply `q_earth2body` and desired rates are `omega_body_radps` (not `q_earth2body_des` or `omega_des_radps`). Similarly, in the Navigation bus `NAVb`, they are `q_earth2body` and `omega_body_radps` (since `NAVb` inherently represents estimated states).

---

## 3. Configuration Conventions (`cfg.hpp`)

Configuration structures (typically suffixed with `c`, e.g., `NAVc`, `CTLc`, `PID_SCALARc`, `PID_3DOFc`, `GNCc`) specify constants like gains, time-steps, and initial states. They are organized as follows:
*   **Master Config**: All configurations are consolidated into a single centralized configuration file [src/GNC/cfg.hpp](file:///src/GNC/cfg.hpp). This file contains the master configuration struct `GNCc`, which bundles `NAVc` (instance name `navc`), `CTLc` (instance name `ctlc`, containing nested `PID_3DOFc` configs for rate and angle loops), and `GUIc` (instance name `guic`).

---

## 4. Coding & Math Standards

*   **Precision**: Use single-precision floats (`float`, `Eigen::Vector3f`, `Eigen::Quaternionf`) for all physical control-loop quantities to maximize performance on microcontroller hardware. Cast double-precision library types (e.g., from UKF) explicitly using `.cast<float>()`.
*   **Quaternion Naming Convention**: Quaternions must always be named using the explicit format `q_<FRAMEA>2<FRAMEB>` representing the rotation from Frame A to Frame B (e.g., `q_earth2body` for estimated attitude, `q_earth2body_des` for desired attitude, and `q_body2body_des` for the attitude error rotation).
*   **Physical Units in Variable Names**: Every variable representing a physical quantity with units must explicitly append the unit as a suffix in lowercase (e.g. `_s` for seconds, `_radps` for radians per second, `_mps2` for meters per second squared, `_deg` for degrees, `_frac` for unitless fractions/ratios). No exceptions. For example, use `dt_s` instead of `dt`.
*   **Descriptive Variable Naming**: Avoid creating short, cryptically named local helper variables (such as `def_s`, `min_m`, `min_s`, `max_s`, `T`, `R`, `P`, `Y`). If a local variable is needed, use a full, descriptive, self-documenting name (e.g. `throttle`, `roll_effort`, `minimum_motor_fraction`). Assign config variables directly to struct members when possible rather than using temporary short-name placeholders.
*   **Enum Branching**: When branching based on `enum` or `enum class` values, use `if/else` if it is a binary choice (either a certain state or any other state) or a unary check. Use `switch` statements only when there are three or more distinct cases to handle. This ensures clean, concise branching for simple checks while maintaining compiler-assisted completeness warnings for complex state machines.
*   **Compilation Checks**: Do not invoke compilation commands (e.g. `pio run` or PlatformIO builds) unless explicitly requested by the user, or after verifying that a valid, compile-worthy entrypoint (such as `main.cpp`) is present in the workspace.
*   **Submodule Data Encapsulation**: To prevent cross-contamination of bus data, GNC submodules must not return the entire `GNCb` bus. Instead, their `update` methods must accept `const GNCb& gnc` (read-only) and return only their designated sub-bus structure (e.g. `NAVb` for `NAV`, `VSMb` for `VSM`, etc.). The master `GNC` coordinator class is responsible for updating/swapping these returned sub-buses back into the main `gnc` bus.
*   **Commit Attribution**: Any commit containing modifications written or assisted by an LLM must explicitly credit the LLM by name in the commit body (e.g., "Assisted by Gemini").
*   **Headers**: Every header file must start with `#pragma once`.
*   **Semicolons**: Struct definitions in C++ must end with a semicolon `;` (e.g., `struct IMU { ... };`).
*   **Styling**: 
    *   Avoid comments in core mathematical or algorithmic blocks; use clean vertical whitespace/line spacing to separate sections of logic instead.
    *   Ensure proper use of namespaces or fully qualified paths (e.g., `Eigen::Vector3f` rather than pulling global namespaces).

---

## 5. LLM Correction & Self-Learning Policy

To ensure continuous improvement and prevent repeating mistakes, the following rule applies to all AI coding agents working on this codebase:
*   **Error Reflection**: Any time the user corrects the agent on a design decision, C++ syntax, project convention, or variable naming, the agent **must** immediately update this [LLM.md](file:///LLM.md) file to document:
    1. The mistake/issue that occurred.
    2. The advised solution or standard.
    3. Rules or constraints to prevent the same issue in the future.

### Correction Log:
*   **Correction #1 (2026-07-08)**: Redundant Bus Member Variables inside Submodule Classes.
    *   *Mistake*: The agent kept a duplicate state variable `NAVb navb;` inside the `NAV` class.
    *   *Advised Solution*: Do not store local copies of bus structs (e.g. `NAVb`, `CTLb`) as class members. All estimations and control efforts must be calculated and written directly onto the fields of the master `GNCb` bus passed to the submodule's `update()` function.
    *   *Action*: Remove any such bus member variables from submodule classes.

*   **Correction #2 (2026-07-08)**: Generalized PID Class and Struct Naming.
    *   *Mistake*: The agent used axis-specific naming (`AXIS_PIDc` / `PID` with `roll`/`pitch`/`yaw` axis classes) which was too restrictive for a generic controller library.
    *   *Advised Solution*: Rename single-dimension PID components to a scalar naming scheme (`PID_scalarc` config struct, `PID_scalar` class). Group 3D/3-axis vector PID controllers under a 3-DOF naming scheme (`PID_3DOFc` config struct, `PID_3DOF` class). Ensure all config and bus instances are consistent with these naming schemes.
    *   *Action*: Replaced `AXIS_PIDc` and `PIDc` with `PID_scalarc` and `PID_3DOFc`, and renamed the `PID` class containing three sub-controllers to `PID_3DOF` comprising three `PID_scalar` sub-instances.

*   **Correction #3 (2026-07-08)**: Casing rules for bus and configuration structure names.
    *   *Mistake*: The agent used lowercase letters in a config struct name (`PID_scalarc`).
    *   *Advised Solution*: All bus and configuration structures must be named in ALL CAPS except for the final `b` or `c` suffix (e.g. `PID_SCALARc` instead of `PID_scalarc`).
    *   *Action*: Renamed `PID_scalarc` to `PID_SCALARc` in `cfg.hpp` and updated all file references.

*   **Correction #4 (2026-07-08)**: Quaternion Naming Conventions.
    *   *Mistake*: The agent used ambiguous naming (e.g., `q_des`, `q_err`) for orientation quaternions.
    *   *Advised Solution*: Always use the naming format `q_<FRAMEA>2<FRAMEB>` to describe the rotation direction clearly (e.g., `q_earth2body_des`, `q_body2body_des`).
    *   *Action*: Documented this naming standard and updated the planning files.

*   **Correction #5 (2026-07-08)**: Redundant Suffixes inside Bus Variables.
    *   *Mistake*: The agent used redundant suffixes like `_des` (e.g. `q_earth2body_des`, `omega_des_radps`) inside the `GUIb` bus.
    *   *Advised Solution*: Never append suffixes like `_des`, `_cmd`, `_est`, or `_meas` to variables in bus structs. The context is already defined by the bus itself (e.g. `GUIb` represents target guidance commands, `NAVb` represents estimated navigation states, `HALb` represents hardware/sensor values).
    *   *Action*: Documented the rule in LLM.md and updated the plan to use `q_earth2body` and `omega_body_radps` inside `GUIb`.

*   **Correction #6 (2026-07-08)**: Missing Units in Variable Names.
    *   *Mistake*: The agent used variable name `dt` instead of `dt_s` for sample time in seconds.
    *   *Advised Solution*: Always append physical unit suffixes to variable names representing physical quantities (e.g., `_s` for seconds).
    *   *Action*: Documented the unit naming convention in LLM.md and updated the plan to use `dt_s` and `time_accumulator_s`.

*   **Correction #7 (2026-07-08)**: Explicit Module-Level Loop Rate Configuration.
    *   *Mistake*: The agent used `cfg_data.angle.pitch.dt_s` (a sub-component PID configuration parameter) to determine the loop rate downsampling condition for the outer attitude loop inside `CTL::update`.
    *   *Advised Solution*: Always define loop rate configurations in a dedicated configuration struct for the parent module (e.g., `CTLc` configuration struct containing `angle_loop_dt_s` for the `CTL` class), rather than borrowing or extracting them from sub-component configurations (like `PID_SCALARc` values).
    *   *Action*: Created `CTLc` configuration struct containing `angle_loop_dt_s`, integrated it into `GNCc`, and updated `CTL.cpp` and `LLM.md`.

*   **Correction #8 (2026-07-08)**: Nested Component Configurations inside Parent Module Configs.
    *   *Mistake*: The agent defined `rate` and `angle` PID configurations directly inside the global `GNCc` structure instead of nesting them inside the control-module-specific `CTLc` configuration structure.
    *   *Advised Solution*: Component configurations (like `PID_3DOFc` rate and angle configs) must be nested within their respective parent module configurations (like `CTLc`) to maintain a modular and hierarchical structure.
    *   *Action*: Nested `rate` and `angle` inside `CTLc` in `cfg.hpp`, and updated `CTL.cpp` and `LLM.md`.

*   **Correction #9 (2026-07-08)**: Use Standard Library for Mathematical Powers.
    *   *Mistake*: The agent used manual variable multiplication (e.g. `input * input * input`) instead of the standard library function (`std::pow`).
    *   *Advised Solution*: Always use C++ standard library functions (like `std::pow` from `<cmath>`) to compute mathematical powers, for clarity and standards compliance.
    *   *Action*: Replaced manual cubic multiplication with `std::pow(input, 3.0f)` inside `GUI.cpp` and included `<cmath>`.

*   **Correction #10 (2026-07-08)**: Enum Branching using Switch Statements.
    *   *Mistake*: The agent used `if-else` statements to branch based on `STATE` and `ATT_MODE` (formerly `CONTROL_MODE`) enums when it should have used `switch` statements for complex states.
    *   *Advised Solution*: Always use `switch` statements when branching based on enum values with three or more cases to enforce compiler-time checks for unhandled cases.
    *   *Action*: Added the rule to LLM.md and updated the plan to use `switch` statements for enum branching.

*   **Correction #11 (2026-07-08)**: Premature Compilation Invocation.
    *   *Mistake*: The agent ran PlatformIO build checks (`pio run`) before verifying that a valid, compile-worthy entrypoint (`main.cpp`) was present in the workspace.
    *   *Advised Solution*: Do not invoke build commands unless explicitly asked by the user or after confirming a compile-worthy entrypoint is present.
    *   *Action*: Added the rule to LLM.md.

*   **Correction #12 (2026-07-08)**: Mismatched Enum Values.
    *   *Mistake*: The agent used a placeholder enum value `STATE::ACTIVE` instead of the actual `STATE` enum class values defined in `bus.hpp` (`STATE::IDLE`, `STATE::RATE`, `STATE::ANGLE`, `STATE::GPS_HOLD`).
    *   *Advised Solution*: Always view and double check actual definitions of enum values in the codebase before implementing branches that check them.
    *   *Action*: Fixed `ALLOC.cpp` to use the correct `STATE` enum values and recorded the correction in `LLM.md`.

*   **Correction #13 (2026-07-08)**: Pragmatic Ternary Checks vs Switch-Case.
    *   *Mistake*: The agent used a verbose switch statement inside `clamp_actuators` to branch on all states, when a simple ternary check for a single state (disarmed) was much cleaner.
    *   *Advised Solution*: Use switch-case blocks for full state machines, but use simple `if-else` or ternary checks when only querying a specific boolean state (e.g. `state == STATE::DISARMED`). Also, renamed `STATE::IDLE` to `STATE::DISARMED` for safety naming clarity.
    *   *Action*: Renamed `STATE::IDLE` to `STATE::DISARMED` in `bus.hpp` and `ALLOC.cpp`, updated `clamp_actuators` to use a ternary condition, and logged the update in `LLM.md`.

*   **Correction #14 (2026-07-08)**: Useless Short Local Variables.
    *   *Mistake*: The agent created cryptic local helper variables (like `def_s`, `min_m`, `T`, `R`, `P`, `Y`) inside `ALLOC.cpp` instead of assigning fields directly or naming variables descriptively.
    *   *Advised Solution*: Short, cryptic variable names do not make code cleaner. Assign config variables directly to destination struct fields where possible, or use fully descriptive, self-documenting local variable names when local variables are necessary.
    *   *Action*: Updated `LLM.md` with the descriptive variable naming standard, and refactored `ALLOC.cpp` to use direct assignments and fully written variable names.

*   **Correction #15 (2026-07-08)**: Attitude Mode Enum Naming.
    *   *Mistake*: The agent used the generic name `CONTROL_MODE` and bus field name `control_mode` instead of the attitude-specific names `ATT_MODE` and `att_mode`.
    *   *Advised Solution*: Use specific names: rename `CONTROL_MODE` to `ATT_MODE` and `control_mode` to `att_mode` to clarify inner-loop attitude states and distinguish them from high-level flight states (`STATE`).
    *   *Action*: Renamed the enum and bus field repo-wide, updated `GUI.cpp` and `CTL.cpp` to use `switch` statements for `ATT_MODE` branching, and updated `LLM.md`.

*   **Correction #16 (2026-07-08)**: Redundant Switch-Case on Binary/Unary Enums.
    *   *Mistake*: The agent used `switch` statements to branch on `ATT_MODE` (only two states) and `ALLOCATOR` (only one state for now).
    *   *Advised Solution*: Do not use `switch` statements for binary or unary enum checks. Use simple `if-else` blocks for binary/unary choices to keep code clean and readable, and reserve `switch` blocks for enums with three or more cases.
    *   *Action*: Updated the rule in `LLM.md` and refactored `GUI.cpp`, `CTL.cpp`, and `ALLOC.cpp` to use `if-else` blocks.

*   **Correction #17 (2026-07-08)**: Loose Bus Modifiability in Submodules.
    *   *Mistake*: The agent initially designed all submodule update signatures to accept and return the entire `GNCb` bus, allowing potential cross-bus write contamination.
    *   *Advised Solution*: Enforce encapsulation: pass `const GNCb&` as a read-only input to submodules, and have them return only their respective sub-buses (e.g. `NAVb`, `VSMb`, etc.), allowing the coordinator to perform swapping.
    *   *Action*: Updated the rule in `LLM.md`, refactored all submodules (`NAV`, `VSM`, `GUI`, `CTL`, `ALLOC`), and implemented the `GNC` master coordinator class.
