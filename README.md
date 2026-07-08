# Chesapeake FSW (Flight Software)

<p align="center">
  <img src="configurator/chesapeake.png" alt="Chesapeake Logo" width="320" />
</p>

Chesapeake 2 is the second-generation flight control software designed for advanced vertical takeoff and vertical landing (VTVL) vehicles and quadcopters. Built on the Seeed Studio Xiao RP2350 platform, it provides real-time guidance, navigation, and control (GNC) capabilities using modular components and a high-rate control loop.

---

## Key Features

* **Swappable Control Allocators**: Supports both Twin-Engine VTVL (using dual thrust-vectoring gimbals and differential throttle) and traditional Quadcopter (QuadX Betaflight mixing) allocation schemes.
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
    * `controllers/`: Attitude PID controller modules
    * `allocation/`: Swappable actuator mixers (Twin-Engine VTVL and QuadX)
    * `filters/`: Raw sensor low-pass filtering
    * `gnc_util/`: Math utilities and 3D coordinate transformations
  * **`gnc_config/`**: Config structures mapping CLI commands to memory variables
  * **`pin_config/`**: Hardware pin assignments
  * **`hardware/`**: Real-time loop rate regulator and RC conversion functions
  * **`tests/`**: Test suites for DShot ESCs, LSM6DSV16X IMU, and ELRS receivers
  * `main.cpp`: Main setup and real-time control loop
* **`configurator/`**: Web Serial UI
  * `index.html`: Dashboard structure
  * `styles.css`: Glassmorphic layout and styling
  * `app.js`: Web serial protocol, input syncing, and Three.js IMU rendering

---

## Hardware & Libraries

* **Microcontroller**: Seeed Studio Xiao RP2350 (RP2040 core framework)
* **Sensor**: ST LSM6DSV16X 6-axis IMU (SPI interface)
* **Receiver**: ELRS Receiver (`AlfredoCRSF` library over Serial1)
* **Actuators**: Servos (`Servo` library) and DShot-compatible ESCs (`PIO_DShot` library)

---

## Getting Started

1. Open this repository in PlatformIO.
2. Select your target environment (e.g., `env:seeed_xiao_rp2350`) and compile/upload the code to the board.
3. Open the `configurator/index.html` tool in a Web Serial-supported browser (Chrome/Edge), connect your device, and start tuning!