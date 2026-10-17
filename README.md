# Two-Wheeled Self-Balancing Robot (ESP32 + MPU6500 + TB6612FNG)

![Status](https://img.shields.io/badge/Status-Work%20In%20Progress-orange)
![MCU](https://img.shields.io/badge/MCU-ESP32%20240MHz-blue)
![Sensor](https://img.shields.io/badge/IMU-MPU6500%20I2C-green)
![Control](https://img.shields.io/badge/Control-Kalman%201D%20%2B%20Dual--Case%20PID-red)

An embedded control systems project implementing a real-time **100Hz Closed-Loop Self-Balancing Robot** on an ESP32 microcontroller. This repository documents an ongoing engineering effort to design, tune, and stabilize an inverted pendulum robot under severe non-ideal mechanical constraints: **encoderless TT DC motors**, **3°–6° plastic gearbox backlash**, and **structural vibration feedback loops**.Because the system's framework cannot be customized, it poses a significant challenge for addressing the issue of inertial forces.

[![Watch the demo video]](https://www.youtube.com/shorts/h5zE2N_4hgM)
---

## 1. System Architecture & Mechanical Specifications

* **Microcontroller:** ESP32 DevKit V1 (Dual-core Xtensa LX6 @ 240MHz)
* **IMU Sensor:** GY-6500 (MPU6500 6-Axis Gyro + Accelerometer via 400kHz Fast-Mode I2C)
* **Motor Driver:** TB6612FNG Dual H-Bridge MOSFET Driver (10kHz PWM, 8-bit resolution)
* **Actuators:** 2x 1:48 Geared TT DC Motors with 66mm rubber wheels (R_w = 33 mm)
* **Chassis Dimensions & Geometry:** 124 x 123 x 52 mm (160 mm total height from ground to top lid, 195 mm wheel track). 
* **Power Supply:** 2S Li-ion Battery (7.4V - 8.4V) mounted strictly at the bottom of the chassis + MP1584EN 5V Buck Converter.

---

## 2. Current Engineering Challenges & Technical Bottlenecks (WIP Analysis)

The primary hurdle in this project is not the theoretical PID mathematics, but overcoming real-world physical limitations that directly impede complete system stabilization. Below is an analysis of the core challenges currently being addressed:

### A. Mechanical Gearbox Backlash (3° - 6° Dead-time)
* **The Problem:** The 1:48 plastic gearboxes on standard TT motors exhibit a significant mechanical backlash (3° - 6° of free play on the output shaft). When the controller reverses motor direction around the 0° equilibrium point, the motor armature spins freely for nearly a full revolution before teeth engagement occurs.
* **Impact on Stabilization:** This creates a *Mechanical Dead-time* (~30 - 50 ms) where the MCU commands voltage changes, but the wheels remain uncoupled from the chassis. During this blind window, gravity pulls the pendulum freely, leading to limit-cycle oscillations and "hammering" shocks against the plastic gear teeth.
* **Mitigation Strategy (In Progress):** Implementing continuous soft-deadzone interpolation (`MOTOR_DEADZONE = 30.0`, `DZ_RAMP = 4.0`) combined with an anti-stutter derivative clamping mechanism to glide smoothly across the free-play gap without triggering high-frequency mechanical resonance.

### B. Chassis Height vs. Center of Mass (CoM) Discrepancy
* **The Problem:** The robot features an upright, thin box design (160 mm height, 52 mm depth), creating a high aspect ratio. However, to keep the system inherently stable, the heaviest components (the 2S Li-ion battery pack and dual motors) are clustered tightly at the very bottom floor of the chassis. 
* **Impact on Stabilization:** While a low CoM slows down the natural free-fall frequency (omega_n approx 12 rad/s), the tall upright shell (124 mm height box) acts like an elastic cantilever beam. Sudden motor corrections twist and flex the chassis structure, feeding high-frequency vibrations back into the control loop.

### C. IMU Placement & Tangential Acceleration "Whip" Noise
* **The Problem:** Initially, mounting the MPU6500 on the top lid of the 160 mm enclosure (L = 127 mm lever arm from the wheel axle) meant that any structural twist or rapid motor correction subjected the sensor to massive tangential (a_t = L * alpha) and centripetal (a_c = L * omega^2) accelerations.
* **Impact on Stabilization:** Motor jerks were falsely interpreted by the accelerometer (`atan2`) as massive physical tilt angles (up to ±6.4° error spikes), forcing the PID loop into an aggressive, self-sustaining jitter loop.
* **Current Status:** The IMU has been successfully relocated to the bottom floor of the chassis (co-planar with the wheel axle and battery CoM) and integrated with a 1D Kalman Filter and a 1st-order low-pass gyro filter (`GYRO_LPF_ALPHA = 0.40`) to drastically clean up the attitude estimation.

### D. Encoderless Drift & Back-EMF Saturation
* **The Problem:** The system lacks wheel encoders, meaning absolute linear speed and position cannot be directly measured. When the robot tilts outward, standard proportional control outputs a fixed PWM. As the motor gains speed, Back-EMF increases, dropping the effective torque.
* **Impact on Stabilization:** The wheels simply roll steadily along with the tilt rather than accelerating past the Center of Mass, causing the robot to drift continuously in one direction until it falls.
* **Mitigation Strategy (In Progress):** Fine-tuning the custom **Two-Case Control Algorithm** (Case 1 for quiet equilibrium near setpoint; Case 2 for progressive error-persistence detection that injects a `fallback_boost` ramp after 5 consecutive cycles to force a "Run-Back" recovery).

---

## 3. Project Roadmap & Development Milestones

- [x] Hardware assembly, 2S Li-ion power distribution, and 10kHz LEDC PWM driver configuration.
- [x] Register-level MPU6500 I2C driver implementation, 500-sample calibration, and 1D Kalman filter.
- [x] Mechanical redesign: Relocating MPU6500 from the top lid to the bottom floor to eliminate whip noise.
- [x] Soft-deadzone smoothing and Anti-Stutter D-term management to handle 1:48 gear backlash.
- [x] Development of the **Two-Case Fallback Control Logic** to counteract encoderless unidirectional drift.
- [ ] **Current Focus (WIP):** Empirical tuning of `FALLBACK_STEP_PWM` and `BALANCE_WINDOW_X` parameters on high-friction floor surfaces to eliminate remaining micro-stutters during dynamic direction changes.
- [ ] **Planned:** Integration of Bluetooth SPP telemetry for live graphical tuning of PID gains via serial plotters.

---
## 4. System Architecture Overview

<p align="center">
  <img src="So do he thong.png" alt="System Architecture Overview" width="100%">
</p>

## 5. Repository Structure

```text
esp32-self-balancing-robot/
├── src/
│   └── main.cpp                 # Core firmware (Kalman filter + Dual-Case PID + Motor driver)
├── docs/
│   ├── chassis_blueprint.png    # Mechanical dimensions (160mm height, 127mm IMU lever arm)
│   └── system_spec.md           # Formal mathematical specification and kinematics
├── platformio.ini               # PlatformIO build configuration for ESP32
└── README.md                    # Project documentation and WIP status
