# Chesapeake Flight Control System

Chesapeake is an embedded flight control firmware designed for the Seeed Studio Xiao RP2350 microcontroller. It uses PlatformIO with the Arduino framework, integrating the Eigen library for optimized matrix and vector mathematics.

This document describes the structure and organization of the core source code directory (`src`).

---

## Source Directory Structure (`src/`)

The `src/` folder is divided into three primary directories:
1. **`CONFIGURATOR/`**: Tooling and assets related to ground control / configuration (currently empty).
2. **`HAL/`**: Hardware Abstraction Layer for device-specific sensor and peripheral drivers (currently empty).
3. **`GNC/`**: Guidance, Navigation, and Control (core flight software logic).

```
src/
├── CONFIGURATOR/          # Configurator assets (currently empty)
├── HAL/                   # Hardware Abstraction Layer (currently empty)
└── GNC/                   # Guidance, Navigation, and Control
    ├── bus.hpp            # Central definition of state and communication buses
    ├── cfg.hpp            # Central configuration file (GNCc config master)
    ├── NAV/               # Navigation (State Estimation)
    │   ├── NAV.hpp        # NAV class declaration (uses UKF and GNCb interface)
    │   └── NAV.cpp        # NAV class implementation
    ├── CTL/               # Control Algorithms
    │   ├── CTL.hpp        # CTL class declaration (conforms to GNCb interface)
    │   ├── CTL.cpp        # CTL class implementation
    │   └── PID/           # PID Controller Sub-module
    │       ├── PID_3DOF.hpp # PID_3DOF & PID_scalar class declarations
    │       └── PID_3DOF.cpp # PID_3DOF & PID_scalar class implementations
    └── UTIL/              # Math Utilities
        ├── quat_util.hpp  # Quaternion math utility declarations
        └── quat_util.cpp  # Quaternion math utility implementations
```

---

## Core Components Description

### 1. Data Buses & Configurations (`src/GNC/`)
*   **[bus.hpp](file:///src/GNC/bus.hpp)**: Defines the standard communication interfaces (buses) passing data between blocks (`HALb`, `NAVb`, `CTLb`, and the master `GNCb` struct).
*   **[cfg.hpp](file:///src/GNC/cfg.hpp)**: Defines the master configuration structure `GNCc` which bundles `NAVc` (navigation constants), `CTLc` (control loop constants), and `PID_3DOFc` (consisting of three `PID_SCALARc` structures for roll, pitch, and yaw control, configured for rate and angle control loops).

### 2. State Estimation (`src/GNC/NAV/`)
The Navigation module handles attitude and state estimation:
*   **[NAV.hpp](file:///src/GNC/NAV/NAV.hpp)** & **[NAV.cpp](file:///src/GNC/NAV/NAV.cpp)**: Declares and implements the `NAV` class, which uses an Unscented Kalman Filter (`UKF` library) to estimate the vehicle's orientation and body rates. It implements the standard interface:
    ```cpp
    GNCb update(GNCb gnc);
    ```

### 3. Flight Control (`src/GNC/CTL/`)
The Control module processes state estimations and pilot inputs to calculate motor and servo command signals:
*   **[CTL.hpp](file:///src/GNC/CTL/CTL.hpp)** & **[CTL.cpp](file:///src/GNC/CTL/CTL.cpp)**: Declares and implements the control loop coordinator. It handles both standard Rate Control and multi-rate cascaded Attitude Control (Angle Loop -> Rate Loop).
*   **PID Sub-module (`src/GNC/CTL/PID/`)**:
    *   **[PID_3DOF.hpp](file:///src/GNC/CTL/PID/PID_3DOF.hpp)** & **[PID_3DOF.cpp](file:///src/GNC/CTL/PID/PID_3DOF.cpp)**: Implements the single-axis scalar controller `PID_scalar` and the 3-axis vector controller `PID_3DOF` (using three `PID_scalar` instances for roll, pitch, and yaw) with integral anti-windup and output constraint limiting, configured using `PID_3DOFc` and `PID_SCALARc` structs.

### 4. Utilities (`src/GNC/UTIL/`)
Shared mathematical operations used across multiple modules:
*   **[quat_util.hpp](file:///src/GNC/UTIL/quat_util.hpp)** & **[quat_util.cpp](file:///src/GNC/UTIL/quat_util.cpp)**: Custom quaternion-to-Euler conversion function (`quat_to_euler`) that handles singularities and outputs angles in the full $[-\pi, \pi]$ range.
