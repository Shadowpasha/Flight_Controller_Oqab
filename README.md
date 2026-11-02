# Flight_Controller_Oqab

ESP32-based modular flight controller for quadcopters (F330 frame, Quad-X configuration). Features a 250 Hz deterministic attitude loop, 50 Hz altitude/horizontal Kalman filtering, 10 Hz GPS dead reckoning navigation, and runtime-selectable control algorithms.

## 🚀 Key Features

* **Multi-Algorithm Attitude Controllers**:
  * **PID**: Cascaded Angle (P) & Rate (PID) loops.
  * **SMC**: Sliding Mode Control with boundary layer.
  * **ADRC**: Linear Active Disturbance Rejection Control (LADRC with ESO).
  * **MDBCI**: Modified Dual Boundary Conditional Integration Backstepping (*ISA Transactions 2020*).
  * **MDBCIS**: MDBCI + Jerk-Mitigating Switched Controller.
  * **VIABC**: Variable Integral Adaptive Backstepping (*IEEE Access 2024*).
  * **CDI / CVI / NOI**: Conditional, Conventional, and No-Integration Backstepping variants.
* **Sensor Suite**:
  * **IMU**: MPU6050 (6-DOF Accelerometer + Gyroscope) at 250 Hz.
  * **Barometer**: BMP180 via I2C bypass.
  * **Compass / Magnetometer**: GY-87 (QMC5883P / HMC5883L).
  * **Altitude**: Sharp GP2Y0A IR Rangefinder (Pin 34) + BMP180 hybrid 3-state Kalman filter.
  * **GPS**: NEO-6M / NEO-7M / NEO-M8N / BN-220 via TinyGPS++ at 9600 baud (UART2: RX=16, TX=17).
* **Communications & Telemetry**:
  * **USB Serial & Classic Bluetooth SPP** (`BluetoothSerial`, device name: `F330_Flight_Controller`).
  * Live dead-reckoning telemetry stream with structured status flags.
  * Interactive CLI for runtime tuning, arming/disarming, mode switching, and parameter management.

## 🔌 Pin Configuration

| Component | Pin (ESP32) | Notes |
| :--- | :--- | :--- |
| **ESC 1 (Front Right, CCW)** | GPIO 14 | 250 Hz PWM |
| **ESC 2 (Rear Right, CW)** | GPIO 27 | 250 Hz PWM |
| **ESC 3 (Rear Left, CCW)** | GPIO 26 | 250 Hz PWM |
| **ESC 4 (Front Left, CW)** | GPIO 25 | 250 Hz PWM |
| **Sharp IR Distance** | GPIO 34 | ADC1 Channel 6 |
| **GPS RX** | GPIO 16 | Connects to GPS TX (9600 baud) |
| **GPS TX** | GPIO 17 | Connects to GPS RX |
| **I2C SDA / SCL** | GPIO 21 / 22 | IMU, Baro, Compass |

## 🛠 Project Structure

```text
balance_controller/
├── balance_controller.ino  # Main setup, loop scheduler, and state machine
├── Config.h                # Hardware pinouts, loop timing, safety ceilings
├── FlightState.h           # Drone state, mission stages, sensor data structs
├── Controllers.h / .cpp    # PID, SMC, ADRC, Backstepping implementations
├── StateEstimator.h / .cpp # Attitude filter, 3-state altitude EKF, GPS projection
├── Sensors.h / .cpp        # Modular hardware drivers (MPU6050, Sharp, BMP, Compass, GPS)
├── Motors.h / .cpp         # ESC signal generation and Quad-X motor mixer
├── Comms.h / .cpp          # Serial/Bluetooth telemetry and interactive CLI
├── Parameters.h / .cpp     # Non-volatile parameter storage (NVS / Flash)
└── tools/
    ├── test_gps/           # Standalone 1 Hz GPS telemetry & auto-baud tool
    ├── recover_gps/        # U-blox factory reset & NMEA 9600 baud restoration script
    └── test_bluetooth/     # Standalone ESP32 Bluetooth Classic SPP test script
```

## 💻 Quick Start

1. Open `balance_controller.ino` in the Arduino IDE.
2. Select your ESP32 board (`ESP32 Dev Module`).
3. Compile and flash via USB.
4. Open the Serial Monitor (or connect via Bluetooth Terminal) at **115200 baud**.
5. Available CLI commands:
   * `arm` / `disarm` - Motor control toggle
   * `ctrl <pid|smc|adrc|mdbci|mdbcis|viabc>` - Switch active attitude controller
   * `gps` - View live GPS satellite and communication status
   * `status` - Full flight controller health report
   * `param show` - Display stored tuning parameters
