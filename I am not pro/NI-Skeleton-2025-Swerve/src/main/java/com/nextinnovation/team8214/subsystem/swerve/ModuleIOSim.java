package com.nextinnovation.team8214.subsystem.swerve;

import com.nextinnovation.team8214.Config;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ModuleIOSim implements ModuleIO {
  private final DCMotorSim driveSim;
  private final DCMotorSim steerSim;

  private final PIDController drivePid;
  private final PIDController steerPid;

  private final SlewRateLimiter driveVoltageLimiter = new SlewRateLimiter(2.5);

  private double driveAppliedVoltageVolt = 0.0;
  private double steerAppliedVoltageVolt = 0.0;

  private double kv;

  ModuleIOSim() {
    var driveGains = SwerveConfig.getDriveGains();
    var steerGains = SwerveConfig.getSteerGains();

    drivePid = new PIDController(driveGains.kp(), 0.0, driveGains.kd(), Config.LOOP_PERIOD_SEC);
    kv = driveGains.kv();
    steerPid = new PIDController(steerGains.kp(), 0.0, steerGains.kd(), Config.LOOP_PERIOD_SEC);

    var driveMotor = DCMotor.getKrakenX60Foc(1);
    var steerMotor = DCMotor.getKrakenX60Foc(1);

    driveSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(driveMotor, 0.025, SwerveConfig.DRIVE_REDUCTION),
            driveMotor);
    steerSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(steerMotor, 0.004, SwerveConfig.STEER_REDUCTION),
            steerMotor);

    steerPid.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    if (DriverStation.isDisabled()) {
      stop();
    }

    if (DriverStation.isDisabled()) {
      setDriveVoltage(driveVoltageLimiter.calculate(driveAppliedVoltageVolt));
    } else {
      driveVoltageLimiter.reset(driveAppliedVoltageVolt);
    }

    driveSim.update(Config.LOOP_PERIOD_SEC);
    steerSim.update(Config.LOOP_PERIOD_SEC);

    inputs.driveMotorConnected = true;
    inputs.drivePositionRad = driveSim.getAngularPositionRad();
    inputs.driveVelRadPerSec = driveSim.getAngularVelocityRadPerSec();
    inputs.driveOutputVoltageVolt = driveAppliedVoltageVolt;
    inputs.driveSupplyCurrentAmp = Math.abs(driveSim.getCurrentDrawAmps());

    inputs.steerEncoderConnected = true;
    inputs.steerMotorConnected = true;
    inputs.steerAbsPosition = new Rotation2d(steerSim.getAngularPositionRad());
    inputs.steerVelRadPerSec = steerSim.getAngularVelocityRadPerSec();
    inputs.steerOutputVoltageVolt = steerAppliedVoltageVolt;
    inputs.steerSupplyCurrentAmp = Math.abs(steerSim.getCurrentDrawAmps());
  }

  @Override
  public void setDriveVoltage(double voltageVolt) {
    driveAppliedVoltageVolt = MathUtil.clamp(voltageVolt, -12.0, 12.0);
    driveSim.setInputVoltage(driveAppliedVoltageVolt);
  }

  private void setSteerVoltage(double voltageVolt) {
    steerAppliedVoltageVolt = MathUtil.clamp(voltageVolt, -12.0, 12.0);
    steerSim.setInputVoltage(steerAppliedVoltageVolt);
  }

  @Override
  public void setDrivePdf(double kp, double kd, double kv, double ks) {
    drivePid.setPID(kp, 0.0, kd);
    this.kv = kv;
  }

  @Override
  public void setSteerPdf(double kp, double kd, double ks) {
    steerPid.setPID(kp, 0.0, kd);
  }

  @Override
  public void setDriveVelocity(double velRadPerSec) {
    var driveAppliedVoltageVolt =
        drivePid.calculate(driveSim.getAngularVelocityRadPerSec(), velRadPerSec)
            + Units.radiansToRotations(velRadPerSec) * kv;
    driveAppliedVoltageVolt = MathUtil.clamp(driveAppliedVoltageVolt, -12.0, 12.0);

    setDriveVoltage(driveAppliedVoltageVolt);
  }

  @Override
  public void setSteerPosition(double angleRad) {
    setSteerVoltage(steerPid.calculate(steerSim.getAngularPositionRad(), angleRad));
  }

  @Override
  public void stop() {
    setDriveVoltage(0.0);
    setSteerVoltage(0.0);
  }
}
