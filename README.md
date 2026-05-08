# ESP32-S3 Drone Flight Controller

This repository contains a professional-grade flight control system for a quadcopter, developed on the **ESP32-S3** platform using **PlatformIO** and the Arduino framework. The architecture leverages FreeRTOS to manage high-frequency control loops and digital communication protocols for motor control.

## 🚀 System Architecture

### 1. Dual-Core Task Management
The firmware utilizes a dual-loop system to prioritize flight stability over background tasks:
*   **Fast Loop (Core 1):** A dedicated FreeRTOS task running at **100Hz (10ms)**. It handles mission-critical operations: IMU data acquisition, orientation estimation, and the PID control pipeline.
*   **Slow Loop (Core 0):** Manages non-critical telemetry, battery voltage monitoring, and serial debugging to prevent interference with the flight dynamics.

### 2. State Estimation & IMU Integration
*   **Sensor:** ICM-20948 9-axis IMU over I2C ($400\text{kHz}$ clock).
*   **Concurrency:** Implements `SemaphoreHandle_t` mutexes to ensure thread-safe I2C transactions.
*   **Fusion Algorithm:** A complementary filter fuses gyroscope data with accelerometer and magnetometer readings.
    *   **Roll/Pitch:** Accelerometer-corrected gyro integration.
    *   **Yaw:** Magnetometer-corrected integration with "shortest path" wrap-around logic to prevent 0/360° boundary snaps.

### 3. Advanced PID Control Theory
The PID implementation focuses on flight smoothness and hardware longevity:
*   **Derivative-on-Measurement:** The D-term is calculated based on the change in sensor values rather than the error. This eliminates "derivative kick" during sudden stick movements.
*   **D-Term Low Pass Filter:** Suppresses high-frequency noise and vibrations to prevent ESC/motor overheating.
*   **Anti-Windup:** Integral limits are strictly enforced to maintain control authority at throttle limits.

### 4. Modes of Operation
*   **Angle Mode (Self-Leveling):** Stick inputs define the target tilt angle; the drone levels itself when sticks are released.
*   **Rate Mode (Acro):** Stick inputs define the angular velocity (DPS), allowing for full manual control and maneuvers.

### 5. DShot Digital Protocol
Optimized for **HAKRC 4-in-1 ESCs**, the system communicates via **DShot600**:
*   **Hardware:** Utilizes the ESP32 RMT (Remote Control) peripheral for precise timing.
*   **Benefits:** High-speed digital communication, CRC error checking, and no requirement for ESC throttle calibration.

## 🔌 Hardware Pinout (ESP32-S3)

| Component | ESP32-S3 GPIO | Note |
| :--- | :--- | :--- |
| **I2C SDA** | 8 | IMU Data Line |
| **I2C SCL** | 9 | IMU Clock Line |
| **Motor 1 (S1)** | 4 | DShot Channel 0 |
| **Motor 2 (S2)** | 5 | DShot Channel 1 |
| **Motor 3 (S3)** | 6 | DShot Channel 2 |
| **Motor 4 (S4)** | 7 | DShot Channel 3 |
| **VBAT Sense** | ADC Pin | Via Voltage Divider (Max 3.3V) |

## 🛠 Project Structure
*   `src/main.cpp`: Main firmware, FreeRTOS task configuration, and motor mixer.
*   `include/imu.h`: Sensor drivers and complementary filter logic.
*   `include/PID.h`: PID controller class with measurement-based derivative.
*   `include/ModeOfOperation.h`: Flight mode switching logic.

## 📦 Dependencies
Add the following to your `platformio.ini`:
```ini
lib_deps =
    sparkfun/SparkFun ICM-20948 Arduino Library﻿# Drone_Project
