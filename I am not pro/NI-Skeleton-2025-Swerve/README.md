# NI Skeleton Swerve

This project is based on NI-Skeleton and provides a basic template for swerve only robot.

## Features

- ✅ Complete Swerve drive control
- ✅ Dynamic Swerve acceleration limiting
- ✅ High-frequency odometry (250Hz CANivore)
- ✅ Teleop and trajectory modes
- ✅ Gyro heading stabilization
- ✅ Simulation support
- ✅ AdvantageKit logging integration

## Getting Started

### Prerequisites

1. **Update RoboRIO Firmware**
   - Use NI FRC Game Tools to update RoboRIO to latest version

2. **Configure RoboRIO Team Number**
   - Set team number in NI FRC Imaging Tool

### Hardware Configuration

3. **Update CTRE Device Firmware**
   - Use Phoenix Tuner X to update TalonFX motors and Pigeon2 gyro

4. **Configure CANivore Bus Name**
   - Ensure CANivore bus name matches project config (default: `"chassis"`)
   - Reference: `src/main/java/com/nextinnovation/team8214/Ports.java`

5. **Set CTRE Device CAN IDs**
   - Configure CAN IDs as follows:
     - **Pigeon2 (Gyro)**: ID 0
     - **FL Module**: Drive ID 1, Steer ID 2, CANcoder ID 3
     - **BL Module**: Drive ID 4, Steer ID 5, CANcoder ID 6
     - **BR Module**: Drive ID 7, Steer ID 8, CANcoder ID 9
     - **FR Module**: Drive ID 10, Steer ID 11, CANcoder ID 12
   - Reference: `src/main/java/com/nextinnovation/team8214/Ports.java`

### Calibration & Configuration

6. **Calibrate CANcoder Offset**
   - Use Phoenix Tuner X CANcoder Calibrator for each module
   - Update offset values in `src/main/java/com/nextinnovation/team8214/subsystem/swerve/SwerveConfig.java`
   - Example: `getCancoderConfig(-0.487548828125)`

7. **Calibrate Pigeon2 Gyro**
   - Place robot on level surface
   - Lift left side of chassis to tilt right until Phoenix Tuner X confirms calibration

8. **Verify Gear Ratio**
   - Confirm gear ratio matches actual modules (current: SDS MK5n R2: 6.82:1)
   - Update `DRIVE_REDUCTION` and `STEER_REDUCTION` in `SwerveConfig.java` if different

9. **Configure Motor Direction**
   - Verify Drive and Steer motor direction in `SwerveConfig.java`

10. **Set Maximum Velocity**
    - Calculate theoretical max velocity from gear ratio and wheel diameter
    - Update `MAX_TRANSLATION_VEL_METER_PER_SEC` in `SwerveConfig.java`

### Deploy Code

11. **Deploy to Robot**
    - Use WPILib or IntelliJ IDEA deploy feature

## Troubleshooting

**Uncontrolled Rotation During Movement**
- If the chassis rotates uncontrollably while moving forward, verify Drive motor polarity is correct
- Check `SwerveConfig.java` for proper motor direction configuration
- **Diagnostic**: If the robot works correctly in simulation mode but fails on real hardware, the issue is motor configuration only
