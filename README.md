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
    ├── GUI/               # Guidance Submodule (RC Expo & stick to target mappings)
    │   ├── GUI.hpp        # GUI class declaration
    │   └── GUI.cpp        # GUI class implementation
    └── ALLOC/             # Actuator Allocation Sub-module (mixer & safety clamps)
        ├── ALLOC.hpp      # ALLOC class declaration
        └── ALLOC.cpp      # ALLOC class implementation
```

---

## Core Components Description

### 1. Data Buses & Configurations (`src/GNC/`)
*   **[bus.hpp](file:///src/GNC/bus.hpp)**: Defines the standard communication interfaces (buses) passing data between blocks (`HALb`, `NAVb`, `CTLb`, and the master `GNCb` struct).
*   **[cfg.hpp](file:///src/GNC/cfg.hpp)**: Defines the master configuration structure `GNCc` which bundles `NAVc` (navigation constants), `CTLc` (control loop constants, enclosing rate and angle `PID_3DOFc` loop parameters), and `GUIc` (guidance expo and scale parameters).

### 2. State Estimation (`src/GNC/NAV/`)
The Navigation module handles attitude and state estimation:
*   **[NAV.hpp](file:///src/GNC/NAV/NAV.hpp)** & **[NAV.cpp](file:///src/GNC/NAV/NAV.cpp)**: Interfaces with the double-precision UKF sensor fusion library to update estimations and extract the body up-vector and roll/pitch Euler angles onto the bus.

### 3. Flight Control (`src/GNC/CTL/`)
The Control module processes state estimations and pilot inputs to calculate motor and servo command signals:
*   **[CTL.hpp](file:///src/GNC/CTL/CTL.hpp)** & **[CTL.cpp](file:///src/GNC/CTL/CTL.cpp)**: Declares and implements the control loop coordinator. It handles both standard Rate Control and multi-rate cascaded Attitude Control (Angle Loop -> Rate Loop).
*   **PID Sub-module (`src/GNC/CTL/PID/`)**:
    *   **[PID_3DOF.hpp](file:///src/GNC/CTL/PID/PID_3DOF.hpp)** & **[PID_3DOF.cpp](file:///src/GNC/CTL/PID/PID_3DOF.cpp)**: Implements the single-axis scalar controller `PID_scalar` and the 3-axis vector controller `PID_3DOF` (using three `PID_scalar` instances for roll, pitch, and yaw) with integral anti-windup and output constraint limiting, configured using `PID_3DOFc` and `PID_SCALARc` structs.

### 4. Guidance (`src/GNC/GUI/`)
Handles pilot stick expo calculations and maps outputs to control/rate commands based on VSM state:
*   **[GUI.hpp](file:///src/GNC/GUI/GUI.hpp)** & **[GUI.cpp](file:///src/GNC/GUI/GUI.cpp)**: Implements the Guidance class.

### 5. Actuator Allocation (`src/GNC/ALLOC/`)
Translates throttle and raw multi-axis control efforts into motor and servo commands:
*   **[ALLOC.hpp](file:///src/GNC/ALLOC/ALLOC.hpp)** & **[ALLOC.cpp](file:///src/GNC/ALLOC/ALLOC.cpp)**: Implements standard mixing algorithms (e.g. QUAD X) and applies standardized safety limits (e.g., disarmed motor shutdowns and servo centering) using `ALLOCc` constraints.
