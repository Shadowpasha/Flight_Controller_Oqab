# Flight_Controller_Oqab

ESP32-based modular flight controller for multirotor UAVs and quadcopters (Quad-X configuration). Features a 250 Hz deterministic attitude loop, 50 Hz altitude/horizontal Kalman filtering, 10 Hz GPS dead reckoning navigation, and runtime-selectable control algorithms.

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
  * **Optical Flow & ToF LiDAR**: Mateksys 3901-L0X / 3901-L1X via MSPv2 on UART2 (RX=16, TX=17 @ 115200 baud) for gyro-derotated horizontal velocity and primary altitude estimation.
  * **Barometer & Compass**: BMP180 / GY-87 (QMC5883P / HMC5883L) via I2C.
  * **GPS (Optional)**: NEO-6M / NEO-7M / NEO-M8N / BN-220 via TinyGPS++ on UART2.
* **PX4-Style Outer Control & Flight Management**:
  * **Altitude & Hover**: 50 Hz Z position P -> velocity PID (`MPC_Z_CLIMB = 0.18 m/s`, `MPC_Z_DESC = 0.35 m/s`), bumpless liftoff handoff (`MPC_THR_HOVER = 1450 us`, `TAKEOFF_THROTTLE = 1480 us`), and toggleable in-flight hover learning (`MPC_HOVER_LEARN`).
  * **Separate Descent Rates**: Normal landing (`land` / hover timeout) at `MPC_RAMP_DN = 35 us/s` (12 cm/s) and emergency fast landing (`start` toggle in flight / `stop` / ceiling guard) at `MPC_RAMP_EMG = 100 us/s` (40 cm/s).
  * **Gentle F330 Position Hold**: Optical flow position hold activates once airborne above `POS_HOLD_MIN_AGL_CM` (9.0 cm over ground) with gentle F330 defaults (`MPC_XY_P = 0.60`, `MPC_XY_VEL_P = 1.50`, `MPC_XY_MAX_TILT = 4.0 deg`).
* **Communications, Telemetry & Dashboard**:
  * **USB Serial & Classic Bluetooth SPP** (`BluetoothSerial`, device name: `Oqab_Flight_Controller`).
  * Live dead-reckoning telemetry stream and interactive CLI for runtime tuning, arming, mode switching, and Flash NVS parameter management.
  * Companion **Tauri/React Dashboard** in `dashboard/` for 3D attitude visualization, live telemetry charting, and flow derotation inspection.

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

1. Open `balance_controller/balance_controller.ino` in the Arduino IDE.
2. Select your ESP32 board (`ESP32 Dev Module`).
3. Compile and flash via USB.
4. Open the Serial Monitor (or connect via Bluetooth Terminal) at **115200 baud**.
5. Available CLI commands:
   * `arm` / `disarm` - Motor control toggle
   * `start` - Arm & begin autonomous takeoff/hover/land mission; send `start` again in flight for emergency fast landing (`MPC_RAMP_EMG`, 100 us/s)
   * `land` / `stop` - Normal controlled descent (`MPC_RAMP_DN`, 35 us/s) vs emergency landing (`stop`)
   * `hover learn <on|off>` - Enable or disable automatic hover throttle learning (`MPC_HOVER_LEARN`)
   * `ctrl <pid|smc|adrc|mdbci|mdbcis|viabc>` - Switch active attitude controller
   * `flow <on|off|toggle>` / `gps <on|off|toggle>` - Enable or disable Optical Flow / GPS subsystems (saved to Flash)
   * `status` - Full flight controller health report
   * `param show` / `param set <NAME> <VAL>` / `param reset` - Inspect, update, or reset Flash NVS parameters

See [docs/oqab_control_estimation_architecture.tex](docs/oqab_control_estimation_architecture.tex) for the full mathematical model, state machine, and estimator equations.
