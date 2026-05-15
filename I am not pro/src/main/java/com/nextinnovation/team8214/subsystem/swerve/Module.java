package com.nextinnovation.team8214.subsystem.swerve;

import com.nextinnovation.team8214.Config;
import com.nextinnovation.team8214.util.Alert;
import com.nextinnovation.team8214.util.LoggedTunableNumber;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.Logger;

class Module {
  private static final LoggedTunableNumber driveKp =
      new LoggedTunableNumber(Config.LiveDebugGroup.SWERVE.toString(), "Swerve/Module/DriveKp");
  private static final LoggedTunableNumber driveKd =
      new LoggedTunableNumber(Config.LiveDebugGroup.SWERVE.toString(), "Swerve/Module/DriveKd");
  private static final LoggedTunableNumber driveKv =
      new LoggedTunableNumber(Config.LiveDebugGroup.SWERVE.toString(), "Swerve/Module/DriveKv");
  private static final LoggedTunableNumber driveKs =
      new LoggedTunableNumber(Config.LiveDebugGroup.SWERVE.toString(), "Swerve/Module/DriveKs");
  private static final LoggedTunableNumber steerKp =
      new LoggedTunableNumber(Config.LiveDebugGroup.SWERVE.toString(), "Swerve/Module/SteerKp");
  private static final LoggedTunableNumber steerKd =
      new LoggedTunableNumber(Config.LiveDebugGroup.SWERVE.toString(), "Swerve/Module/SteerKd");
  private static final LoggedTunableNumber steerKs =
      new LoggedTunableNumber(Config.LiveDebugGroup.SWERVE.toString(), "Swerve/Module/SteerKs");

  static {
    var driveSlot = SwerveConfig.getX2iDriveTalonConfig().Slot0;
    driveKp.initDefault(driveSlot.kP);
    driveKd.initDefault(driveSlot.kD);
    driveKv.initDefault(driveSlot.kV);
    driveKs.initDefault(driveSlot.kS);

    var steerSlot = SwerveConfig.getX2iSteerTalonNoEncoderConfig().Slot0;
    steerKp.initDefault(steerSlot.kP);
    steerKd.initDefault(steerSlot.kD);
    steerKs.initDefault(steerSlot.kS);
  }

  private final String name;

  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();

  private final Alert driveMotorOfflineAlert;
  private final Alert steerMotorOfflineAlert;
  private final Alert steerEncoderOfflineAlert;

  Module(ModuleIO io, String name) {
    this.io = io;
    this.name = "Module" + name;

    driveMotorOfflineAlert =
        new Alert(this.name + " drive motor offline!", Alert.AlertType.WARNING);
    steerMotorOfflineAlert =
        new Alert(this.name + " steer motor offline!", Alert.AlertType.WARNING);
    steerEncoderOfflineAlert =
        new Alert(this.name + " steer encoder offline!", Alert.AlertType.WARNING);
  }

  void updateInputs() {
    io.updateInputs(inputs);
    Logger.processInputs("Swerve/" + name, inputs);

    // Update gains when changed during live debugging
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> io.setDrivePdf(driveKp.get(), driveKd.get(), driveKv.get(), driveKs.get()),
        driveKp,
        driveKd,
        driveKv,
        driveKs);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> io.setSteerPdf(steerKp.get(), steerKd.get(), steerKs.get()),
        steerKp,
        steerKd,
        steerKs);

    // Display alerts
    driveMotorOfflineAlert.set(!inputs.driveMotorConnected);
    steerMotorOfflineAlert.set(!inputs.steerMotorConnected);
    steerEncoderOfflineAlert.set(!inputs.steerEncoderConnected);
  }

  void setState(SwerveModuleState state) {
    io.setDriveVelocity(state.speedMetersPerSecond / SwerveConfig.WHEEL_RADIUS_METER);
    io.setSteerPosition(state.angle.getRadians());
  }

  SwerveModuleState getState() {
    return new SwerveModuleState(
        inputs.driveVelRadPerSec * SwerveConfig.WHEEL_RADIUS_METER, inputs.steerAbsPosition);
  }

  double getDrivePositionRad() {
    return inputs.drivePositionRad;
  }

  double getDriveVelRadPerSec() {
    return inputs.driveVelRadPerSec;
  }

  double getSteerPositionRad() {
    return inputs.steerAbsPosition.getRadians();
  }

  void setDriveVoltage(double voltageVolt) {
    io.setDriveVoltage(voltageVolt);
  }

  void stop() {
    io.stop();
  }
}
