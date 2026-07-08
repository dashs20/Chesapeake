# Chesapeake FSW (Flight Software)

<p align="center">
  <img src="configurator/chesapeake.png" alt="Chesapeake Logo" width="320" />
</p>

Chesapeake is a flight control software designed for advanced vertical takeoff and vertical landing (VTVL) vehicles and quadcopters. Built on the Seeed Studio Xiao RP2350 platform, it provides real-time guidance, navigation, and control (GNC) capabilities using modular components and a high-rate control loop.

For repository-wide context and developer guidelines, see **[LLM.md](file:///C:/Users/dashs/OneDrive/Documents/PlatformIO/Projects/Chesapeake/LLM.md)**.

---

## Key Features

* **Swappable Control Allocators**: Supports both Twin-Engine VTVL and traditional Quadcopter (QuadX) allocation schemes.
* **PID Attitude Rate Controllers**: Fully adjustable PID controllers for Roll, Pitch, and Yaw attitude rate control loops with integrated anti-windup clamping.
* **First-Order Lowpass Filters**: Low-latency signal filtering on IMU gyro readings to reduce vibration and sensor noise.
* **Dynamic Mounting Orientation**: Computes full 3D Euler coordinate rotations for the IMU to handle physical mounting offsets on the vehicle.
* **EEPROM State Persistence**: Integrates a custom configuration manager that loads and saves calibration and control parameters across reboots.
* **Interactive Web Configurator**: A Maryland Calvert/Crossland flag-themed web interface built with Web Serial API and Three.js 3D visualizer for live tuning and attitude monitoring.
* **Built-in CLI Terminal**: Command-line interface accessible via standard Serial to query, set, or default GNC parameters on-the-fly.

---

## Project Structure

* **`src/`**: Core flight software source code
  * **`gnc/`**: Guidance, Navigation, and Control algorithms
    * **`controllers/`**: Attitude PID controller modules ([src/gnc/controllers/pid.hpp](file:///C:/Users/dashs/OneDrive/Documents/PlatformIO/Projects/Chesapeake/src/gnc/controllers/pid.hpp))
    * **`allocation/`**: Swappable actuator mixers ([src/gnc/allocation/alloc.hpp](file:///C:/Users/dashs/OneDrive/Documents/PlatformIO/Projects/Chesapeake/src/gnc/allocation/alloc.hpp))
    * **`filters/`**: Raw sensor low-pass filtering ([src/gnc/filters/lowpass_filter.hpp](file:///C:/Users/dashs/OneDrive/Documents/PlatformIO/Projects/Chesapeake/src/gnc/filters/lowpass_filter.hpp))
    * **`gnc_util/`**: Math utilities and 3D coordinate transformations ([src/gnc/gnc_util/gnc_util.hpp](file:///C:/Users/dashs/OneDrive/Documents/PlatformIO/Projects/Chesapeake/src/gnc/gnc_util/gnc_util.hpp))
  * **`gnc_config/`**: Config structures mapping CLI commands to memory variables ([src/gnc_config/gnc_config.hpp](file:///C:/Users/dashs/OneDrive/Documents/PlatformIO/Projects/Chesapeake/src/gnc_config/gnc_config.hpp))
  * **`pin_config/`**: Hardware pin assignments ([src/pin_config/pin_config.hpp](file:///C:/Users/dashs/OneDrive/Documents/PlatformIO/Projects/Chesapeake/src/pin_config/pin_config.hpp))
  * **`hardware/`**: Real-time loop rate regulator and RC conversion functions ([src/hardware/hardware_util.hpp](file:///C:/Users/dashs/OneDrive/Documents/PlatformIO/Projects/Chesapeake/src/hardware/hardware_util.hpp))
  * **`tests/`**: Test suites for DShot ESCs, LSM6DSV16X IMU, and ELRS receivers
  * **[src/main.cpp](file:///C:/Users/dashs/OneDrive/Documents/PlatformIO/Projects/Chesapeake/src/main.cpp)**: Main setup and real-time control loop
* **`configurator/`**: Web Serial UI
  * **[configurator/index.html](file:///C:/Users/dashs/OneDrive/Documents/PlatformIO/Projects/Chesapeake/configurator/index.html)**: Dashboard structure
  * **[configurator/app.js](file:///C:/Users/dashs/OneDrive/Documents/PlatformIO/Projects/Chesapeake/configurator/app.js)**: Web serial protocol, input syncing, and Three.js IMU rendering
  * **[configurator/styles.css](file:///C:/Users/dashs/OneDrive/Documents/PlatformIO/Projects/Chesapeake/configurator/styles.css)**: Layout styling

---

## Hardware & Libraries

* **Microcontroller**: Seeed Studio Xiao RP2350 (RP2040 core framework)
* **Sensor**: ST LSM6DSV16X 6-axis IMU (SPI interface)
* **Receiver**: ELRS Receiver (`AlfredoCRSF` library over hardware serial defined in [src/pin_config/pin_config.cpp](file:///C:/Users/dashs/OneDrive/Documents/PlatformIO/Projects/Chesapeake/src/pin_config/pin_config.cpp))
* **Actuators**: Servos (`Servo` library) and DShot-compatible ESCs (`PIO_DShot` library)

---

## Getting Started

1. Open this repository in PlatformIO.
2. Select your target environment defined in **[platformio.ini](file:///C:/Users/dashs/OneDrive/Documents/PlatformIO/Projects/Chesapeake/platformio.ini)** and compile/upload the code to the board.
3. Open the **[configurator/index.html](file:///C:/Users/dashs/OneDrive/Documents/PlatformIO/Projects/Chesapeake/configurator/index.html)** tool in a Web Serial-supported browser (Chrome/Edge), connect your device, and start tuning!