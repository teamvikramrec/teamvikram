# ESP32 TVC Flight Controller

A lightweight active stabilization system for model rockets using Thrust Vector Control (TVC). This firmware runs on an ESP32, processing data from an MPU6050 IMU to adjust the angle of the rocket motor gimbal via two servos, keeping the vehicle vertical during ascent.

## 🚀 Features
* **Sensor Fusion:** Implements a **Complementary Filter** (Alpha 0.98) to combine Accelerometer and Gyroscope data for drift-free, responsive angle estimation.
* **PD Control Loop:** Proportional-Derivative controller to correct orientation errors.
* **Servo Constraints:** Safety limits (`SERVO_LIMIT`) prevent the TVC mount from binding or over-extending.
* **High Performance:** Runs at approximately 200Hz loop frequency.

## 🛠️ Hardware Requirements
* **Microcontroller:** ESP32 Development Board
* **IMU:** MPU6050 (6-axis Accelerometer/Gyroscope)
* **Actuators:** 2x Servos (e.g., SG90, MG90S) connected to a TVC Mount
* **Power:** 5V BEC or Battery appropriate for servos

## 🔌 Wiring / Pinout

| Component | Pin Label | ESP32 Pin |
| :--- | :--- | :--- |
| **Servo Pitch (Y)** | Signal | `GPIO 18` |
| **Servo Roll (X)** | Signal | `GPIO 19` |
| **MPU6050** | SDA | `GPIO 21` |
| **MPU6050** | SCL | `GPIO 22` |
| **MPU6050** | VCC | 3.3V or 5V |
| **MPU6050** | GND | GND |

> **Note:** Ensure the Servos and ESP32 share a common Ground (GND).

## 📦 Software Dependencies
You must install the following library via the Arduino Library Manager:

1.  **ESP32Servo** by Kevin Harrington

## ⚙️ Configuration & Tuning

Parameters in the code can be adjusted to fit your specific airframe:

### 1. Servo Alignment
* **`SERVO_CENTER` (Default: 90):** The servo angle where the motor is perfectly straight.
* **`SERVO_LIMIT` (Default: 12):** The maximum number of degrees the motor can tilt in either direction.

### 2. PID Tuning
The controller uses a PD loop to maintain stability.
* **`Kp` (Proportional):** Reacts to the current angle error. Increase if the rocket is sluggish to correct; decrease if it oscillates rapidly.
* **`Kd` (Derivative):** Reacts to the *rate* of change. Acts as a "damper" to prevent overshooting.

```cpp
// Control gains (TUNE CAREFULLY)
float Kp = 1.5;   // Start LOW and increase
float Kd = 0.05;  // Dampening factor