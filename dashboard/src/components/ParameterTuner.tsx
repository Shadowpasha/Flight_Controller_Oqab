import React, { useState } from 'react';
import { Sliders, Save, RefreshCw, RotateCcw, Check } from 'lucide-react';
import { serialService } from '../services/serialService';

export interface TunableParam {
  name: string;
  type: 'float' | 'int' | 'bool' | 'enum';
  value: number;
  defaultValue: number;
  min: number;
  max: number;
  step?: number;
  description: string;
  category:
    | 'Altitude & Vertical Control'
    | 'Position & Navigation'
    | 'Attitude Control'
    | 'Mission & Flowchart'
    | 'Safety & Failsafe'
    | 'Optical Flow Subsystem'
    | 'Sensor Calibration'
    | 'State Estimator & Filters'
    | 'System & Sensors';
  options?: { value: number; label: string }[];
}

interface Props {
  isArmed: boolean;
}

export const ParameterTuner: React.FC<Props> = ({ isArmed }) => {
  const [params, setParams] = useState<TunableParam[]>([
    {
      name: 'MPC_THR_ARM_MIN',
      type: 'int',
      value: 1080.0,
      defaultValue: 1080.0,
      min: 1020.0,
      max: 1200.0,
      step: 1,
      description: 'Minimum Armed Idle Throttle [us]',
      category: 'Altitude & Vertical Control',
    },
    {
      name: 'MPC_THR_HOVER',
      type: 'int',
      value: 1320.0,
      defaultValue: 1320.0,
      min: 1150.0,
      max: 1600.0,
      step: 1,
      description: 'Nominal Hover Throttle [us]',
      category: 'Altitude & Vertical Control',
    },
    {
      name: 'MPC_THR_LAND',
      type: 'int',
      value: 1140.0,
      defaultValue: 1140.0,
      min: 1080.0,
      max: 1300.0,
      step: 1,
      description: 'Landing Floor Throttle [us]',
      category: 'Altitude & Vertical Control',
    },
    {
      name: 'MIS_STOP_LAND',
      type: 'bool',
      value: 1.0,
      defaultValue: 1.0,
      min: 0.0,
      max: 1.0,
      step: 1,
      description: 'Stop Triggers Controlled Land (0/1)',
      category: 'Mission & Flowchart',
    },
    {
      name: 'MPC_RAMP_UP',
      type: 'float',
      value: 70.0,
      defaultValue: 70.0,
      min: 10.0,
      max: 150.0,
      step: 1.0,
      description: 'Climb Ramp Rate [us/s]',
      category: 'Altitude & Vertical Control',
    },
    {
      name: 'MPC_RAMP_DN',
      type: 'float',
      value: 35.0,
      defaultValue: 35.0,
      min: 10.0,
      max: 100.0,
      step: 1.0,
      description: 'Descent Ramp Rate [us/s]',
      category: 'Altitude & Vertical Control',
    },
    {
      name: 'MPC_Z_TARGET',
      type: 'float',
      value: 100.0,
      defaultValue: 100.0,
      min: 15.0,
      max: 180.0,
      step: 1.0,
      description: 'Target Hover Height [cm]',
      category: 'Altitude & Vertical Control',
    },
    {
      name: 'MPC_Z_P',
      type: 'float',
      value: 1.0,
      defaultValue: 1.0,
      min: 0.1,
      max: 5.0,
      step: 0.01,
      description: 'Altitude Position P-gain',
      category: 'Altitude & Vertical Control',
    },
    {
      name: 'MPC_Z_VEL_P',
      type: 'float',
      value: 1.8,
      defaultValue: 1.8,
      min: 0.2,
      max: 8.0,
      step: 0.1,
      description: 'Altitude Velocity P-gain',
      category: 'Altitude & Vertical Control',
    },
    {
      name: 'MPC_Z_VEL_I',
      type: 'float',
      value: 0.50,
      defaultValue: 0.50,
      min: 0.0,
      max: 3.0,
      step: 0.01,
      description: 'Altitude Velocity I-gain',
      category: 'Altitude & Vertical Control',
    },
    {
      name: 'MPC_Z_VEL_D',
      type: 'float',
      value: 0.04,
      defaultValue: 0.04,
      min: 0.0,
      max: 0.5,
      step: 0.001,
      description: 'Altitude Velocity D-gain',
      category: 'Altitude & Vertical Control',
    },
    {
      name: 'MPC_Z_CLIMB',
      type: 'float',
      value: 0.18,
      defaultValue: 0.18,
      min: 0.05,
      max: 0.8,
      step: 0.01,
      description: 'Max Climb Speed [m/s]',
      category: 'Altitude & Vertical Control',
    },
    {
      name: 'MPC_Z_DESC',
      type: 'float',
      value: 0.35,
      defaultValue: 0.35,
      min: 0.05,
      max: 1.2,
      step: 0.01,
      description: 'Max Descent Speed [m/s]',
      category: 'Altitude & Vertical Control',
    },
    {
      name: 'MPC_ALT_HOLD',
      type: 'bool',
      value: 1.0,
      defaultValue: 1.0,
      min: 0.0,
      max: 1.0,
      step: 1,
      description: 'Altitude Hold Enable (0/1)',
      category: 'Altitude & Vertical Control',
    },
    {
      name: 'AUTO_LAND_EN',
      type: 'bool',
      value: 1.0,
      defaultValue: 1.0,
      min: 0.0,
      max: 1.0,
      step: 1,
      description: 'Auto Landing Enable (0/1)',
      category: 'Altitude & Vertical Control',
    },
    {
      name: 'HOVER_TIME_S',
      type: 'int',
      value: 6.0,
      defaultValue: 6.0,
      min: 1.0,
      max: 300.0,
      step: 1,
      description: 'Auto Hover Duration [s]',
      category: 'Altitude & Vertical Control',
    },
    {
      name: 'MPC_POS_HOLD',
      type: 'bool',
      value: 1.0,
      defaultValue: 1.0,
      min: 0.0,
      max: 1.0,
      step: 1,
      description: 'GPS Position Hold Enable (0/1)',
      category: 'Position & Navigation',
    },
    {
      name: 'MPC_XY_P',
      type: 'float',
      value: 1.2,
      defaultValue: 1.2,
      min: 0.1,
      max: 5.0,
      step: 0.01,
      description: 'Horizontal Position P-gain',
      category: 'Position & Navigation',
    },
    {
      name: 'MPC_XY_VEL_P',
      type: 'float',
      value: 3.5,
      defaultValue: 3.5,
      min: 0.5,
      max: 10.0,
      step: 0.1,
      description: 'Horizontal Velocity P-gain',
      category: 'Position & Navigation',
    },
    {
      name: 'MPC_XY_VEL_I',
      type: 'float',
      value: 1.0,
      defaultValue: 1.0,
      min: 0.0,
      max: 5.0,
      step: 0.01,
      description: 'Horizontal Velocity I-gain',
      category: 'Position & Navigation',
    },
    {
      name: 'MPC_XY_VEL_D',
      type: 'float',
      value: 0.15,
      defaultValue: 0.15,
      min: 0.0,
      max: 1.0,
      step: 0.01,
      description: 'Horizontal Velocity D-gain',
      category: 'Position & Navigation',
    },
    {
      name: 'MPC_XY_MAX_VEL',
      type: 'float',
      value: 1.5,
      defaultValue: 1.5,
      min: 0.2,
      max: 5.0,
      step: 0.01,
      description: 'Max Return Speed [m/s]',
      category: 'Position & Navigation',
    },
    {
      name: 'MPC_XY_MAX_TILT',
      type: 'float',
      value: 12.0,
      defaultValue: 12.0,
      min: 3.0,
      max: 30.0,
      step: 0.1,
      description: 'Max Tilt Angle [deg]',
      category: 'Position & Navigation',
    },
    {
      name: 'MC_ROLL_P',
      type: 'float',
      value: 2.0,
      defaultValue: 2.0,
      min: 0.1,
      max: 10.0,
      step: 0.1,
      description: 'Cascade Roll Angle P',
      category: 'Attitude Control',
    },
    {
      name: 'MC_ROLLRATE_P',
      type: 'float',
      value: 0.625,
      defaultValue: 0.625,
      min: 0.05,
      max: 5.0,
      step: 0.01,
      description: 'Roll Rate P',
      category: 'Attitude Control',
    },
    {
      name: 'MC_ROLLRATE_I',
      type: 'float',
      value: 2.1,
      defaultValue: 2.1,
      min: 0.0,
      max: 10.0,
      step: 0.1,
      description: 'Roll Rate I',
      category: 'Attitude Control',
    },
    {
      name: 'MC_ROLLRATE_D',
      type: 'float',
      value: 0.0088,
      defaultValue: 0.0088,
      min: 0.0,
      max: 0.1,
      step: 0.001,
      description: 'Roll Rate D',
      category: 'Attitude Control',
    },
    {
      name: 'MC_PITCH_P',
      type: 'float',
      value: 2.0,
      defaultValue: 2.0,
      min: 0.1,
      max: 10.0,
      step: 0.1,
      description: 'Cascade Pitch Angle P',
      category: 'Attitude Control',
    },
    {
      name: 'MC_PITCHRATE_P',
      type: 'float',
      value: 0.625,
      defaultValue: 0.625,
      min: 0.05,
      max: 5.0,
      step: 0.01,
      description: 'Pitch Rate P',
      category: 'Attitude Control',
    },
    {
      name: 'MC_PITCHRATE_I',
      type: 'float',
      value: 2.1,
      defaultValue: 2.1,
      min: 0.0,
      max: 10.0,
      step: 0.1,
      description: 'Pitch Rate I',
      category: 'Attitude Control',
    },
    {
      name: 'MC_PITCHRATE_D',
      type: 'float',
      value: 0.0088,
      defaultValue: 0.0088,
      min: 0.0,
      max: 0.1,
      step: 0.001,
      description: 'Pitch Rate D',
      category: 'Attitude Control',
    },
    {
      name: 'MC_YAWRATE_P',
      type: 'float',
      value: 2.0,
      defaultValue: 2.0,
      min: 0.1,
      max: 10.0,
      step: 0.1,
      description: 'Yaw Rate P',
      category: 'Attitude Control',
    },
    {
      name: 'SYS_USE_GPS',
      type: 'bool',
      value: 0,
      defaultValue: 0,
      min: 0.0,
      max: 1.0,
      step: 1,
      description: 'Enable GPS Subsystem (0/1)',
      category: 'System & Sensors',
    },
    {
      name: 'SYS_USE_FLOW',
      type: 'bool',
      value: 1,
      defaultValue: 1,
      min: 0.0,
      max: 1.0,
      step: 1,
      description: 'Enable Matek Optical Flow (0/1)',
      category: 'Optical Flow Subsystem',
    },
    {
      name: 'SENS_FLOW_ROT',
      type: 'float',
      value: 0.0,
      defaultValue: 0.0,
      min: 0.0,
      max: 360.0,
      step: 1.0,
      description: 'Flow Sensor Mounting Yaw [deg]',
      category: 'Optical Flow Subsystem',
    },
    {
      name: 'FLOW_RAD_SCALE',
      type: 'float',
      value: 0.0038,
      defaultValue: 0.0038,
      min: 0.0005,
      max: 0.0100,
      step: 0.0001,
      description: 'Flow Angular Res [rad/count]',
      category: 'Optical Flow Subsystem',
    },
    {
      name: 'FLOW_DELAY_MS',
      type: 'float',
      value: 25.0,
      defaultValue: 25.0,
      min: 0.0,
      max: 100.0,
      step: 1.0,
      description: 'Flow Latency Delay [ms]',
      category: 'Optical Flow Subsystem',
    },
    {
      name: 'FLOW_POS_X',
      type: 'float',
      value: -0.04,
      defaultValue: -0.04,
      min: -0.50,
      max: 0.50,
      step: 0.01,
      description: 'Flow Offset from CM X (Fwd) [m]',
      category: 'Optical Flow Subsystem',
    },
    {
      name: 'FLOW_POS_Y',
      type: 'float',
      value: 0.01,
      defaultValue: 0.01,
      min: -0.50,
      max: 0.50,
      step: 0.01,
      description: 'Flow Offset from CM Y (Right) [m]',
      category: 'Optical Flow Subsystem',
    },
    {
      name: 'FLOW_POS_Z',
      type: 'float',
      value: 0.03,
      defaultValue: 0.03,
      min: -0.50,
      max: 0.50,
      step: 0.01,
      description: 'Flow Offset from CM Z (Down) [m]',
      category: 'Optical Flow Subsystem',
    },
    {
      name: 'IMU_POS_X',
      type: 'float',
      value: -0.02,
      defaultValue: -0.02,
      min: -0.50,
      max: 0.50,
      step: 0.01,
      description: 'IMU Offset from CM X (Fwd) [m]',
      category: 'System & Sensors',
    },
    {
      name: 'IMU_POS_Y',
      type: 'float',
      value: 0.02,
      defaultValue: 0.02,
      min: -0.50,
      max: 0.50,
      step: 0.01,
      description: 'IMU Offset from CM Y (Right) [m]',
      category: 'System & Sensors',
    },
    {
      name: 'SYS_CTRL_TYPE',
      type: 'enum',
      value: 0,
      defaultValue: 0,
      min: 0.0,
      max: 8.0,
      step: 1,
      description: 'Active Controller (0=PID..3=MDBCI..8=VIABC)',
      category: 'System & Sensors',
      options: [
        { value: 0, label: '0: PID' },
        { value: 1, label: '1: SMC' },
        { value: 2, label: '2: ADRC' },
        { value: 3, label: '3: MDBCI' },
        { value: 4, label: '4: MDBCIS' },
        { value: 5, label: '5: CDI' },
        { value: 6, label: '6: CVI' },
        { value: 7, label: '7: NOI' },
        { value: 8, label: '8: VIABC' },
      ],
    },
    {
      name: 'SYS_FLIGHT_MOD',
      type: 'enum',
      value: 0.0,
      defaultValue: 0.0,
      min: 0.0,
      max: 2.0,
      step: 1,
      description: 'Flight Mode (0=POS_HOLD, 1=ALT_HOLD, 2=MANUAL)',
      category: 'System & Sensors',
      options: [
        { value: 0, label: '0: POS_HOLD (Pos + Alt Hold)' },
        { value: 1, label: '1: ALT_HOLD (Alt Hold Only)' },
        { value: 2, label: '2: MANUAL (Attitude Only)' },
      ],
    },
    {
      name: 'MAG_DECL_DEG',
      type: 'float',
      value: 0.0,
      defaultValue: 0.0,
      min: -180.0,
      max: 180.0,
      step: 1.0,
      description: 'Magnetic Declination [deg]',
      category: 'System & Sensors',
    },
    {
      name: 'SENS_BOARD_ROT',
      type: 'enum',
      value: 0.0,
      defaultValue: 0.0,
      min: 0.0,
      max: 5.0,
      step: 1,
      description: 'IMU Board Orientation (0=Default..5=Pitch180)',
      category: 'System & Sensors',
      options: [
        { value: 0, label: '0: Default (+Y Fwd, +X Right, +Z Up)' },
        { value: 1, label: '1: None (+X Fwd, +Y Right, +Z Up)' },
        { value: 2, label: '2: Yaw 90° CW' },
        { value: 3, label: '3: Yaw 180° CW' },
        { value: 4, label: '4: Yaw 270° CW' },
        { value: 5, label: '5: Pitch 180° (Inverted)' },
      ],
    },
    {
      name: 'SAFE_MAX_TILT',
      type: 'float',
      value: 40.0,
      defaultValue: 40.0,
      min: 15.0,
      max: 60.0,
      step: 0.1,
      description: 'Crash Protection Max Tilt [deg]',
      category: 'Safety & Failsafe',
    },
    {
      name: 'SAFE_TIMEOUT_S',
      type: 'int',
      value: 30.0,
      defaultValue: 30.0,
      min: 5.0,
      max: 120.0,
      step: 1,
      description: 'Safety Watchdog Disarm Timeout [s]',
      category: 'Safety & Failsafe',
    },
    {
      name: 'SAFE_CEIL_CM',
      type: 'float',
      value: 185.0,
      defaultValue: 185.0,
      min: 50.0,
      max: 200.0,
      step: 1.0,
      description: 'ToF LiDAR Ceiling Protection [cm]',
      category: 'Safety & Failsafe',
    },
    {
      name: 'MIS_TAKEOFF_ALT',
      type: 'float',
      value: 100.0,
      defaultValue: 100.0,
      min: 15.0,
      max: 180.0,
      step: 1.0,
      description: 'Mission Default Takeoff Alt [cm]',
      category: 'Mission & Flowchart',
    },
    {
      name: 'MIS_HOVER_S',
      type: 'int',
      value: 6.0,
      defaultValue: 6.0,
      min: 1.0,
      max: 300.0,
      step: 1,
      description: 'Mission Default Hover Time [s]',
      category: 'Mission & Flowchart',
    },
    {
      name: 'MIS_WAYPOINT_SPD',
      type: 'float',
      value: 1.5,
      defaultValue: 1.5,
      min: 0.2,
      max: 5.0,
      step: 0.01,
      description: 'Mission Waypoint Speed [m/s]',
      category: 'Mission & Flowchart',
    },
    {
      name: 'CAL_ROLL_OFF',
      type: 'float',
      value: 0.0,
      defaultValue: 0.0,
      min: -30.0,
      max: 30.0,
      step: 1.0,
      description: 'Level Accel Roll Offset [deg]',
      category: 'Sensor Calibration',
    },
    {
      name: 'CAL_PITCH_OFF',
      type: 'float',
      value: 0.0,
      defaultValue: 0.0,
      min: -30.0,
      max: 30.0,
      step: 1.0,
      description: 'Level Accel Pitch Offset [deg]',
      category: 'Sensor Calibration',
    },
    {
      name: 'CAL_GYRO_X',
      type: 'float',
      value: 0.0,
      defaultValue: 0.0,
      min: -1.5,
      max: 1.5,
      step: 0.01,
      description: 'Gyro X Bias [rad/s]',
      category: 'Sensor Calibration',
    },
    {
      name: 'CAL_GYRO_Y',
      type: 'float',
      value: 0.0,
      defaultValue: 0.0,
      min: -1.5,
      max: 1.5,
      step: 0.01,
      description: 'Gyro Y Bias [rad/s]',
      category: 'Sensor Calibration',
    },
    {
      name: 'CAL_GYRO_Z',
      type: 'float',
      value: 0.0,
      defaultValue: 0.0,
      min: -1.5,
      max: 1.5,
      step: 0.01,
      description: 'Gyro Z Bias [rad/s]',
      category: 'Sensor Calibration',
    },
    {
      name: 'CAL_ACC_X_OFF',
      type: 'float',
      value: 0.0,
      defaultValue: 0.0,
      min: -5.0,
      max: 5.0,
      step: 0.1,
      description: 'Accel Fwd Offset [m/s²]',
      category: 'Sensor Calibration',
    },
    {
      name: 'CAL_ACC_Y_OFF',
      type: 'float',
      value: 0.0,
      defaultValue: 0.0,
      min: -5.0,
      max: 5.0,
      step: 0.1,
      description: 'Accel Right Offset [m/s²]',
      category: 'Sensor Calibration',
    },
    {
      name: 'CAL_MAG_X',
      type: 'float',
      value: 53.0,
      defaultValue: 53.0,
      min: -3000.0,
      max: 3000.0,
      step: 1.0,
      description: 'Compass Offset X [counts]',
      category: 'Sensor Calibration',
    },
    {
      name: 'CAL_MAG_Y',
      type: 'float',
      value: -1179.0,
      defaultValue: -1179.0,
      min: -3000.0,
      max: 3000.0,
      step: 1.0,
      description: 'Compass Offset Y [counts]',
      category: 'Sensor Calibration',
    },
    {
      name: 'CAL_MAG_Z',
      type: 'float',
      value: 42.5,
      defaultValue: 42.5,
      min: -3000.0,
      max: 3000.0,
      step: 1.0,
      description: 'Compass Offset Z [counts]',
      category: 'Sensor Calibration',
    },
    {
      name: 'EKF_Q_Z',
      type: 'float',
      value: 0.0002,
      defaultValue: 0.0002,
      min: 0.00001,
      max: 0.01,
      step: 0.0001,
      description: 'Vertical KF Pos Process Noise',
      category: 'State Estimator & Filters',
    },
    {
      name: 'EKF_Q_VZ',
      type: 'float',
      value: 0.0035,
      defaultValue: 0.0035,
      min: 0.0001,
      max: 0.05,
      step: 0.0001,
      description: 'Vertical KF Vel Process Noise',
      category: 'State Estimator & Filters',
    },
    {
      name: 'EKF_Q_BA',
      type: 'float',
      value: 0.00003,
      defaultValue: 0.00003,
      min: 0.000001,
      max: 0.001,
      step: 0.0001,
      description: 'Vertical KF Accel Bias Noise',
      category: 'State Estimator & Filters',
    },
    {
      name: 'EKF_R_LIDAR',
      type: 'float',
      value: 0.0025,
      defaultValue: 0.0025,
      min: 0.0001,
      max: 0.05,
      step: 0.0001,
      description: 'ToF LiDAR Measurement Noise R',
      category: 'State Estimator & Filters',
    },
    {
      name: 'ATT_ALPHA',
      type: 'float',
      value: 0.991,
      defaultValue: 0.991,
      min: 0.90,
      max: 0.999,
      step: 0.001,
      description: 'Attitude Filter Alpha Gain',
      category: 'State Estimator & Filters',
    },
    {
      name: 'YAW_ALPHA',
      type: 'float',
      value: 0.995,
      defaultValue: 0.995,
      min: 0.90,
      max: 0.999,
      step: 0.001,
      description: 'Yaw Filter Alpha Gain',
      category: 'State Estimator & Filters',
    },
    {
      name: 'EKF_R_FLOW',
      type: 'float',
      value: 0.02,
      defaultValue: 0.02,
      min: 0.001,
      max: 0.20,
      step: 0.001,
      description: 'Flow Vel Measurement Noise R',
      category: 'State Estimator & Filters',
    },
    {
      name: 'FLOW_Q_THRESH',
      type: 'float',
      value: 35.0,
      defaultValue: 35.0,
      min: 10.0,
      max: 150.0,
      step: 1.0,
      description: 'Flow Min Quality Threshold',
      category: 'Optical Flow Subsystem',
    },
    {
      name: 'FLOW_LPF_HZ',
      type: 'float',
      value: 8.0,
      defaultValue: 8.0,
      min: 1.0,
      max: 25.0,
      step: 0.1,
      description: 'Flow Velocity LPF Cutoff [Hz]',
      category: 'Optical Flow Subsystem',
    },
    {
      name: 'SMC_KP_ROLL',
      type: 'float',
      value: 1.2,
      defaultValue: 1.2,
      min: 0.1,
      max: 10.0,
      step: 0.1,
      description: 'SMC Roll Proportional Gain',
      category: 'Attitude Control',
    },
    {
      name: 'SMC_KSW_ROLL',
      type: 'float',
      value: 0.8,
      defaultValue: 0.8,
      min: 0.05,
      max: 5.0,
      step: 0.01,
      description: 'SMC Roll Switching Gain',
      category: 'Attitude Control',
    },
    {
      name: 'SMC_LAMBDA_R',
      type: 'float',
      value: 1.5,
      defaultValue: 1.5,
      min: 0.1,
      max: 10.0,
      step: 0.1,
      description: 'SMC Roll Sliding Surface Lambda',
      category: 'Attitude Control',
    },
    {
      name: 'SMC_PHI_ROLL',
      type: 'float',
      value: 0.3,
      defaultValue: 0.3,
      min: 0.05,
      max: 2.0,
      step: 0.01,
      description: 'SMC Roll Boundary Layer Phi',
      category: 'Attitude Control',
    },
    {
      name: 'SMC_KP_PITCH',
      type: 'float',
      value: 1.2,
      defaultValue: 1.2,
      min: 0.1,
      max: 10.0,
      step: 0.1,
      description: 'SMC Pitch Proportional Gain',
      category: 'Attitude Control',
    },
    {
      name: 'SMC_KSW_PITCH',
      type: 'float',
      value: 0.8,
      defaultValue: 0.8,
      min: 0.05,
      max: 5.0,
      step: 0.01,
      description: 'SMC Pitch Switching Gain',
      category: 'Attitude Control',
    },
    {
      name: 'SMC_LAMBDA_P',
      type: 'float',
      value: 1.5,
      defaultValue: 1.5,
      min: 0.1,
      max: 10.0,
      step: 0.1,
      description: 'SMC Pitch Sliding Surface Lambda',
      category: 'Attitude Control',
    },
    {
      name: 'SMC_PHI_PITCH',
      type: 'float',
      value: 0.3,
      defaultValue: 0.3,
      min: 0.05,
      max: 2.0,
      step: 0.01,
      description: 'SMC Pitch Boundary Layer Phi',
      category: 'Attitude Control',
    },
    {
      name: 'SMC_KP_YAW',
      type: 'float',
      value: 1.5,
      defaultValue: 1.5,
      min: 0.1,
      max: 10.0,
      step: 0.1,
      description: 'SMC Yaw Proportional Gain',
      category: 'Attitude Control',
    },
    {
      name: 'BS_C1_ROLL',
      type: 'float',
      value: 1.8,
      defaultValue: 1.8,
      min: 0.2,
      max: 10.0,
      step: 0.1,
      description: 'MDBCI Roll Virtual Bandwidth c1',
      category: 'Attitude Control',
    },
    {
      name: 'BS_C2_ROLL',
      type: 'float',
      value: 0.65,
      defaultValue: 0.65,
      min: 0.05,
      max: 5.0,
      step: 0.01,
      description: 'MDBCI Roll Rate Damping c2',
      category: 'Attitude Control',
    },
    {
      name: 'BS_C1_PITCH',
      type: 'float',
      value: 1.8,
      defaultValue: 1.8,
      min: 0.2,
      max: 10.0,
      step: 0.1,
      description: 'MDBCI Pitch Virtual Bandwidth c1',
      category: 'Attitude Control',
    },
    {
      name: 'BS_C2_PITCH',
      type: 'float',
      value: 0.65,
      defaultValue: 0.65,
      min: 0.05,
      max: 5.0,
      step: 0.01,
      description: 'MDBCI Pitch Rate Damping c2',
      category: 'Attitude Control',
    },
    {
      name: 'BS_LAMBDA_M',
      type: 'float',
      value: 1.5,
      defaultValue: 1.5,
      min: 0.1,
      max: 5.0,
      step: 0.01,
      description: 'MDBCI Integral Ceiling lambda_m',
      category: 'Attitude Control',
    },
  ]);

  const [activeCategory, setActiveCategory] = useState<string>('All');
  const [saveSuccess, setSaveSuccess] = useState(false);

  const handleValueChange = (name: string, val: number) => {
    setParams((prev) =>
      prev.map((p) => (p.name === name ? { ...p, value: val } : p))
    );
    serialService.sendCommand(`param set ${name} ${val}`);
  };

  const handleSaveToFlash = () => {
    serialService.sendCommand('param save');
    setSaveSuccess(true);
    setTimeout(() => setSaveSuccess(false), 2000);
  };

  const handleReload = () => {
    serialService.sendCommand('param show');
  };

  const handleResetDefaults = () => {
    if (confirm('Reset all parameters to factory defaults?')) {
      serialService.sendCommand('param reset');
      setParams((prev) => prev.map((p) => ({ ...p, value: p.defaultValue })));
    }
  };

  const categories = [
    'All',
    'Altitude & Vertical Control',
    'Position & Navigation',
    'Attitude Control',
    'Mission & Flowchart',
    'Safety & Failsafe',
    'Optical Flow Subsystem',
    'Sensor Calibration',
    'State Estimator & Filters',
    'System & Sensors',
  ];

  const filteredParams = activeCategory === 'All'
    ? params
    : params.filter((p) => p.category === activeCategory);

  const formatDisplayValue = (p: TunableParam): string => {
    if (p.type === 'int' || p.type === 'enum') return p.value.toFixed(0);
    if (p.type === 'bool') return p.value > 0.5 ? 'ON' : 'OFF';
    if (p.step && p.step < 0.01) return p.value.toFixed(4);
    if (p.step && p.step < 0.1) return p.value.toFixed(3);
    return p.value.toFixed(1);
  };

  return (
    <div className="bg-[#0a0a0a] border border-neutral-800 rounded-lg p-5 flex flex-col gap-4">
      {/* Header Bar */}
      <div className="flex flex-wrap items-center justify-between gap-3 border-b border-neutral-800/80 pb-3">
        <div className="flex items-center gap-2">
          <Sliders className="w-4 h-4 text-red-600" />
          <h2 className="text-xs font-mono font-semibold uppercase tracking-wider text-neutral-200">
            System Parameters ({params.length} Tunable Items)
          </h2>
        </div>

        <div className="flex items-center gap-2 font-mono text-xs">
          <button
            onClick={handleReload}
            className="flex items-center gap-1.5 px-3 py-1.5 rounded bg-neutral-900 hover:bg-neutral-800 border border-neutral-800 text-neutral-300 transition cursor-pointer"
            title="Read live parameters from flight controller"
          >
            <RefreshCw className="w-3.5 h-3.5" />
            <span>Read All</span>
          </button>

          <button
            onClick={handleResetDefaults}
            disabled={isArmed}
            className="flex items-center gap-1.5 px-3 py-1.5 rounded bg-neutral-900 hover:bg-neutral-800 border border-neutral-800 text-neutral-300 transition cursor-pointer disabled:opacity-40 disabled:cursor-not-allowed"
            title="Reset parameters to factory defaults"
          >
            <RotateCcw className="w-3.5 h-3.5 text-neutral-400" />
            <span>Reset Defaults</span>
          </button>

          <button
            onClick={handleSaveToFlash}
            className="flex items-center gap-1.5 px-3.5 py-1.5 rounded bg-red-600 hover:bg-red-700 text-white font-bold transition cursor-pointer shadow-[0_0_10px_rgba(220,38,38,0.3)]"
            title="Commit all modified parameters to ESP32 Flash (NVS)"
          >
            {saveSuccess ? (
              <>
                <Check className="w-3.5 h-3.5" />
                <span>SAVED!</span>
              </>
            ) : (
              <>
                <Save className="w-3.5 h-3.5" />
                <span>Save to Flash</span>
              </>
            )}
          </button>
        </div>
      </div>

      {/* Category Tabs */}
      <div className="flex flex-wrap gap-1.5 border-b border-neutral-800/80 pb-3">
        {categories.map((cat) => {
          const isActive = activeCategory === cat;
          const count = cat === 'All' ? params.length : params.filter((p) => p.category === cat).length;
          return (
            <button
              key={cat}
              onClick={() => setActiveCategory(cat)}
              className={`px-3 py-1 rounded text-xs font-mono transition cursor-pointer flex items-center gap-1.5 ${
                isActive
                  ? 'bg-neutral-800 text-white font-bold border border-neutral-700'
                  : 'bg-neutral-900/60 text-neutral-400 border border-neutral-800/60 hover:text-white'
              }`}
            >
              <span>{cat}</span>
              <span className="text-[10px] opacity-60">({count})</span>
            </button>
          );
        })}
      </div>

      {/* Parameter Cards Grid */}
      <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-3">
        {filteredParams.map((p) => (
          <div
            key={p.name}
            className="bg-black/50 border border-neutral-800/80 rounded p-3 flex flex-col justify-between hover:border-neutral-700 transition"
          >
            <div className="flex items-start justify-between gap-2 mb-2">
              <div>
                <div className="font-mono text-xs font-bold text-white flex items-center gap-1.5">
                  <span>{p.name}</span>
                  {Math.abs(p.value - p.defaultValue) > 0.0001 && (
                    <span className="w-1.5 h-1.5 rounded-full bg-amber-400" title="Modified from default" />
                  )}
                </div>
                <div className="text-[11px] text-neutral-400 mt-0.5 line-clamp-2" title={p.description}>
                  {p.description}
                </div>
              </div>
              <div className="font-mono text-xs font-bold px-2 py-0.5 rounded bg-neutral-900 text-neutral-200 border border-neutral-800 text-right whitespace-nowrap">
                {formatDisplayValue(p)}
              </div>
            </div>

            {/* Input Control Based on Type */}
            <div className="mt-2 pt-2 border-t border-neutral-900">
              {p.type === 'bool' ? (
                <div className="flex gap-2">
                  <button
                    onClick={() => handleValueChange(p.name, 1)}
                    className={`flex-1 py-1 rounded text-xs font-mono font-bold transition cursor-pointer ${
                      p.value > 0.5
                        ? 'bg-emerald-600 text-white'
                        : 'bg-neutral-900 text-neutral-400 hover:text-white border border-neutral-800'
                    }`}
                  >
                    ON (1)
                  </button>
                  <button
                    onClick={() => handleValueChange(p.name, 0)}
                    className={`flex-1 py-1 rounded text-xs font-mono font-bold transition cursor-pointer ${
                      p.value <= 0.5
                        ? 'bg-neutral-800 text-white border border-neutral-700'
                        : 'bg-neutral-900 text-neutral-400 hover:text-white border border-neutral-800'
                    }`}
                  >
                    OFF (0)
                  </button>
                </div>
              ) : p.type === 'enum' && p.options ? (
                <select
                  value={Math.round(p.value)}
                  onChange={(e) => handleValueChange(p.name, parseInt(e.target.value, 10))}
                  className="w-full bg-neutral-900 border border-neutral-800 rounded px-2 py-1 text-xs font-mono text-white focus:outline-none focus:border-red-600"
                >
                  {p.options.map((opt) => (
                    <option key={opt.value} value={opt.value}>
                      {opt.label}
                    </option>
                  ))}
                </select>
              ) : (
                <div className="flex items-center gap-2">
                  <input
                    type="range"
                    min={p.min}
                    max={p.max}
                    step={p.step || 0.1}
                    value={p.value}
                    onChange={(e) => handleValueChange(p.name, parseFloat(e.target.value))}
                    className="flex-1 accent-red-600"
                  />
                  <input
                    type="number"
                    min={p.min}
                    max={p.max}
                    step={p.step || 0.1}
                    value={p.value}
                    onChange={(e) => {
                      const v = parseFloat(e.target.value);
                      if (!isNaN(v)) handleValueChange(p.name, v);
                    }}
                    className="w-16 bg-neutral-900 border border-neutral-800 rounded px-1.5 py-0.5 text-xs font-mono text-white text-right focus:outline-none focus:border-red-600"
                  />
                </div>
              )}

              <div className="flex justify-between text-[10px] font-mono text-neutral-500 mt-1">
                <span>Min: {p.min}</span>
                <span>Def: {p.defaultValue}</span>
                <span>Max: {p.max}</span>
              </div>
            </div>
          </div>
        ))}
      </div>
    </div>
  );
};
