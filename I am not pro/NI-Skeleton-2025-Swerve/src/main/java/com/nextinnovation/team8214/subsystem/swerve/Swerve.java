package com.nextinnovation.team8214.subsystem.swerve;

import static edu.wpi.first.units.Units.Volts;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import com.ctre.phoenix6.SignalLogger;
import com.nextinnovation.team8214.*;
import com.nextinnovation.team8214.Odometry;
import com.nextinnovation.team8214.subsystem.swerve.controller.*;
import com.nextinnovation.team8214.util.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.Arrays;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.function.*;
import lombok.Setter;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

@ExtensionMethod({GeomUtil.class, EqualsUtil.GeomExtensions.class})
public class Swerve extends SubsystemBase {
  private final LoggedTunableNumber maxTiltAccelXMeterPerSec2 =
      new LoggedTunableNumber(
          Config.LiveDebugGroup.SWERVE.toString(), "Swerve/MaxTiltAccelXMeterPerSec2", 20.0);
  private final LoggedTunableNumber maxTiltAccelYMeterPerSec2 =
      new LoggedTunableNumber(
          Config.LiveDebugGroup.SWERVE.toString(), "Swerve/MaxTiltAccelYMeterPerSec2", 20.0);
  private final LoggedTunableNumber maxSkidAccelMeterPerSec2 =
      new LoggedTunableNumber(
          Config.LiveDebugGroup.SWERVE.toString(), "Swerve/MaxSkidAccelMeterPerSec2", 20.0);
  private final LoggedTunableNumber maxForwardAccelMeterPerSec2 =
      new LoggedTunableNumber(
          Config.LiveDebugGroup.SWERVE.toString(), "Swerve/MaxForwardAccelMeterPerSec2", 20.0);

  private enum ControlMode {
    TELEOP,
    TRAJECTORY,
    SYSID,
    WHEEL_RADIUS_CHARACTERIZATION,
  }

  static class SwerveOdometryInputs implements LoggableInputs {
    double[] odometryTimestamps;
    SwerveModulePosition[] odometryFLPositions;
    SwerveModulePosition[] odometryBLPositions;
    SwerveModulePosition[] odometryBRPositions;
    SwerveModulePosition[] odometryFRPositions;
    Rotation2d[] odometryYaws;

    @Override
    public void toLog(LogTable table) {
      table.put("OdometryTimestamps", odometryTimestamps);
      table.put("OdometryFLPositions", odometryFLPositions);
      table.put("OdometryBLPositions", odometryBLPositions);
      table.put("OdometryBRPositions", odometryBRPositions);
      table.put("OdometryFRPositions", odometryFRPositions);

      if (odometryYaws.length != 0 && odometryYaws[0] != null) {
        table.put("OdometryYaws", odometryYaws);
      }
    }

    @Override
    public void fromLog(LogTable table) {
      odometryTimestamps = table.get("OdometryTimestamps", odometryTimestamps);
      odometryFLPositions = table.get("OdometryFLPositions", odometryFLPositions);
      odometryBLPositions = table.get("OdometryBLPositions", odometryBLPositions);
      odometryBRPositions = table.get("OdometryBRPositions", odometryBRPositions);
      odometryFRPositions = table.get("OdometryFRPositions", odometryFRPositions);
      odometryYaws = table.get("OdometryYaws", odometryYaws);
    }
  }

  private ControlMode mode = ControlMode.TELEOP;
  private final Module[] modules = new Module[4];
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final SwerveOdometryInputs swerveOdometryInputs = new SwerveOdometryInputs();
  private final ArrayBlockingQueue<Odometry.WheeledObservation>
      odometryCachedWheeledObservationQueue;

  private SwerveModuleState[] lastGoalModuleStates =
      new SwerveModuleState[] {
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState(),
      };

  private final TeleopController teleopController = new TeleopController();
  private HeadingController headingController = null;
  private TrajectoryController trajectoryController = null;
  private double goalWheelRadiusCharacterizationAngularVel = 0.0;

  @Setter private Supplier<Double> customMaxTiltAccelScale = () -> 1.0;

  private final Alert gyroOfflineAlert = new Alert("Gyro offline!", Alert.AlertType.WARNING);

  public Swerve() {
    switch (Config.MODE) {
      case REAL -> {
        var flModuleIo =
            new ModuleIOKrakenFOC(
                SwerveConfig.FL_MODULE_NAME,
                Ports.Can.FL_DRIVE_MOTOR,
                Ports.Can.FL_STEER_MOTOR,
                Ports.Can.FL_STEER_SENSOR,
                SwerveConfig.FL_MODULE_CONFIG);

        var blModuleIo =
            new ModuleIOKrakenFOC(
                SwerveConfig.BL_MODULE_NAME,
                Ports.Can.BL_DRIVE_MOTOR,
                Ports.Can.BL_STEER_MOTOR,
                Ports.Can.BL_STEER_SENSOR,
                SwerveConfig.BL_MODULE_CONFIG);

        var brModuleIo =
            new ModuleIOKrakenFOC(
                SwerveConfig.BR_MODULE_NAME,
                Ports.Can.BR_DRIVE_MOTOR,
                Ports.Can.BR_STEER_MOTOR,
                Ports.Can.BR_STEER_SENSOR,
                SwerveConfig.BR_MODULE_CONFIG);

        var frModuleIo =
            new ModuleIOKrakenFOC(
                SwerveConfig.FR_MODULE_NAME,
                Ports.Can.FR_DRIVE_MOTOR,
                Ports.Can.FR_STEER_MOTOR,
                Ports.Can.FR_STEER_SENSOR,
                SwerveConfig.FR_MODULE_CONFIG);

        var gyroIOPigeon2 = new GyroIOPigeon2(Ports.Can.CHASSIS_PIGEON);

        gyroIO = gyroIOPigeon2;

        modules[0] = new Module(flModuleIo, SwerveConfig.FL_MODULE_NAME);
        modules[1] = new Module(blModuleIo, SwerveConfig.BL_MODULE_NAME);
        modules[2] = new Module(brModuleIo, SwerveConfig.BR_MODULE_NAME);
        modules[3] = new Module(frModuleIo, SwerveConfig.FR_MODULE_NAME);

        odometryCachedWheeledObservationQueue =
            new PhoenixOdometryThread(
                    flModuleIo.getDrivePosition(),
                    flModuleIo.getSteerAbsPosition(),
                    blModuleIo.getDrivePosition(),
                    blModuleIo.getSteerAbsPosition(),
                    brModuleIo.getDrivePosition(),
                    brModuleIo.getSteerAbsPosition(),
                    frModuleIo.getDrivePosition(),
                    frModuleIo.getSteerAbsPosition(),
                    gyroIOPigeon2.getYaw())
                .start();
      }

      case SIM -> {
        var flModuleIo = new ModuleIOSim();
        var blModuleIo = new ModuleIOSim();
        var brModuleIo = new ModuleIOSim();
        var frModuleIo = new ModuleIOSim();

        gyroIO = new GyroIO() {};

        modules[0] = new Module(flModuleIo, SwerveConfig.FL_MODULE_NAME);
        modules[1] = new Module(blModuleIo, SwerveConfig.BL_MODULE_NAME);
        modules[2] = new Module(brModuleIo, SwerveConfig.BR_MODULE_NAME);
        modules[3] = new Module(frModuleIo, SwerveConfig.FR_MODULE_NAME);

        odometryCachedWheeledObservationQueue =
            new SimOdometryThread(
                    modules[0]::getDrivePositionRad,
                    modules[0]::getSteerPositionRad,
                    modules[1]::getDrivePositionRad,
                    modules[1]::getSteerPositionRad,
                    modules[2]::getDrivePositionRad,
                    modules[2]::getSteerPositionRad,
                    modules[3]::getDrivePositionRad,
                    modules[3]::getSteerPositionRad)
                .start();
      }

      default -> {
        gyroIO = new GyroIO() {};

        modules[0] = new Module(new ModuleIO() {}, SwerveConfig.FL_MODULE_NAME);
        modules[1] = new Module(new ModuleIO() {}, SwerveConfig.BL_MODULE_NAME);
        modules[2] = new Module(new ModuleIO() {}, SwerveConfig.BR_MODULE_NAME);
        modules[3] = new Module(new ModuleIO() {}, SwerveConfig.FR_MODULE_NAME);

        odometryCachedWheeledObservationQueue = new ArrayBlockingQueue<>(20);
      }
    }

    Odometry.getInstance().addTrajectoryVel(new Twist2d());
  }

  public Transform3d getRobotToSwerveTransform() {
    return new Transform3d(
        0.0, 0.0, 0.0, new Rotation3d(Math.PI / 2, Math.PI / 2 * 0, 3 * Math.PI / 2));
  }

  private void updateInputs() {
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Swerve/Gyro", gyroInputs);

    for (var module : modules) {
      module.updateInputs();
    }

    var odometrySampleArray =
        odometryCachedWheeledObservationQueue.toArray(Odometry.WheeledObservation[]::new);
    odometryCachedWheeledObservationQueue.clear();

    var sampleNum = odometrySampleArray.length;
    swerveOdometryInputs.odometryTimestamps = new double[sampleNum];
    swerveOdometryInputs.odometryFLPositions = new SwerveModulePosition[sampleNum];
    swerveOdometryInputs.odometryBLPositions = new SwerveModulePosition[sampleNum];
    swerveOdometryInputs.odometryBRPositions = new SwerveModulePosition[sampleNum];
    swerveOdometryInputs.odometryFRPositions = new SwerveModulePosition[sampleNum];
    swerveOdometryInputs.odometryYaws = new Rotation2d[sampleNum];

    for (int i = 0; i < sampleNum; i++) {
      swerveOdometryInputs.odometryTimestamps[i] = odometrySampleArray[i].timestamp();
      swerveOdometryInputs.odometryFLPositions[i] = odometrySampleArray[i].wheelPositions()[0];
      swerveOdometryInputs.odometryBLPositions[i] = odometrySampleArray[i].wheelPositions()[1];
      swerveOdometryInputs.odometryBRPositions[i] = odometrySampleArray[i].wheelPositions()[2];
      swerveOdometryInputs.odometryFRPositions[i] = odometrySampleArray[i].wheelPositions()[3];
      swerveOdometryInputs.odometryYaws[i] = odometrySampleArray[i].yaw();
    }

    Logger.processInputs("Swerve/Odometry", swerveOdometryInputs);
  }

  @Override
  public void periodic() {
    updateInputs();

    gyroOfflineAlert.set(!gyroInputs.connected);

    var odometry = Odometry.getInstance();
    for (int i = 0; i < swerveOdometryInputs.odometryTimestamps.length; i++) {
      var wheeledObservation =
          new Odometry.WheeledObservation(
              swerveOdometryInputs.odometryTimestamps[i],
              new SwerveModulePosition[] {
                swerveOdometryInputs.odometryFLPositions[i],
                swerveOdometryInputs.odometryBLPositions[i],
                swerveOdometryInputs.odometryBRPositions[i],
                swerveOdometryInputs.odometryFRPositions[i],
              },
              swerveOdometryInputs.odometryYaws[i]);
      odometry.addWheeledObservation(wheeledObservation);
    }

    var currentVel = getVel();
    odometry.addRobotCentricVel(currentVel.toTwist2d());

    if (DriverStation.isDisabled()) {
      clearHeadingGoal();
    }

    var goalVel = new ChassisSpeeds();

    if (mode != ControlMode.TELEOP) {
      teleopController.setInput(0.0, 0.0, 0.0, false);
    }

    switch (mode) {
      case TELEOP -> {
        goalVel = teleopController.update();
        if (headingController != null) {
          teleopController.resetHeadingMaintainerSetpointToCurrent();
          goalVel.omegaRadiansPerSecond = headingController.update();
        }
      }
      case TRAJECTORY -> {
        if (trajectoryController != null) {
          teleopController.resetHeadingMaintainerSetpointToCurrent();
          goalVel = trajectoryController.update();
          if (headingController != null) {
            goalVel.omegaRadiansPerSecond = headingController.update();
          }
        }
      }

      case WHEEL_RADIUS_CHARACTERIZATION ->
          goalVel = new ChassisSpeeds(0.0, 0.0, goalWheelRadiusCharacterizationAngularVel);
    }

    Logger.recordOutput("Swerve/ControlMode", mode);

    if (mode == ControlMode.SYSID) {
      return;
    }

    Logger.recordOutput("Swerve/RawGoalVel", goalVel);

    // Desaturate
    var rawGoalModuleStates = SwerveConfig.SWERVE_KINEMATICS.toSwerveModuleStates(goalVel);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        rawGoalModuleStates, SwerveConfig.MAX_TRANSLATION_VEL_METER_PER_SEC);
    goalVel = SwerveConfig.SWERVE_KINEMATICS.toChassisSpeeds(rawGoalModuleStates);

    // 1690 Orbit accel limitation
    if (mode != ControlMode.TRAJECTORY) {
      goalVel = applyAccelLimitation(currentVel, goalVel);
    }

    // Dynamics compensation
    goalVel = ChassisSpeeds.discretize(goalVel, Config.LOOP_PERIOD_SEC);

    // Use last goal angle for module if chassis want stop completely
    var goalModuleStates = SwerveConfig.SWERVE_KINEMATICS.toSwerveModuleStates(goalVel);
    if (goalVel.toTwist2d().epsilonEquals(new Twist2d())) {
      for (int i = 0; i < modules.length; i++) {
        goalModuleStates[i].angle = lastGoalModuleStates[i].angle;
        goalModuleStates[i].speedMetersPerSecond = 0.0;
      }
    }

    var optimizedGoalModuleStates = new SwerveModuleState[4];

    for (int i = 0; i < modules.length; i++) {
      // Optimize setpoints
      optimizedGoalModuleStates[i] = goalModuleStates[i];
      optimizedGoalModuleStates[i].optimize(modules[i].getState().angle);
      modules[i].setState(optimizedGoalModuleStates[i]);
    }

    lastGoalModuleStates = goalModuleStates;

    Logger.recordOutput("Swerve/SwerveStates/GoalModuleStates", goalModuleStates);
    Logger.recordOutput("Swerve/FinalGoalVel", goalVel);
    Logger.recordOutput("Swerve/SwerveStates/OptimizedGoalModuleStates", optimizedGoalModuleStates);
  }

  @AutoLogOutput(key = "Swerve/RobotCentricVel")
  public ChassisSpeeds getVel() {
    var vel = SwerveConfig.SWERVE_KINEMATICS.toChassisSpeeds(getModuleStates());
    if (gyroInputs.connected) {
      vel.omegaRadiansPerSecond = gyroInputs.yawVelocityRadPerSec;
    }

    return vel;
  }

  @AutoLogOutput(key = "Swerve/ModuleStates")
  public SwerveModuleState[] getModuleStates() {
    return Arrays.stream(modules).map(Module::getState).toArray(SwerveModuleState[]::new);
  }

  public void setTeleopInput(double x, double y, double omega, boolean wantEscape) {
    if (DriverStation.isTeleopEnabled()) {
      mode = ControlMode.TELEOP;
      teleopController.setInput(x, y, omega, wantEscape);
    }
  }

  public void disableTeleopControllerHeadingMaintainer() {
    teleopController.disableHeadingMaintainer();
  }

  public void setTrajectory(Trajectory<SwerveSample> trajectory) {
    if (DriverStation.isAutonomousEnabled()) {
      disableTeleopControllerHeadingMaintainer();
      mode = ControlMode.TRAJECTORY;
      trajectoryController = new TrajectoryController(trajectory);
    }
  }

  public void clearTrajectory() {
    trajectoryController = null;
    mode = ControlMode.TELEOP;
    Odometry.getInstance().addTrajectoryVel(new Twist2d());
  }

  public void setHeadingGoal(Supplier<Rotation2d> goalHeadingSupplier) {
    headingController = new HeadingController(goalHeadingSupplier);
  }

  public void clearHeadingGoal() {
    headingController = null;
  }

  @AutoLogOutput(key = "Swerve/HasTrajectoryDone")
  public boolean hasTrajectoryDone() {
    return trajectoryController != null && trajectoryController.hasDone();
  }

  @AutoLogOutput(key = "Swerve/AtHeadingGoal")
  public boolean atHeadingGoal() {
    return headingController != null && headingController.atGoal();
  }

  public void clearCustomMaxTiltAccelFactor() {
    customMaxTiltAccelScale = () -> 1.0;
  }

  public double[] getWheelRadiusCharacterizationPosition() {
    return Arrays.stream(modules).mapToDouble(Module::getDrivePositionRad).toArray();
  }

  public void setGoalWheelRadiusCharacterizationAngularVel(double angularVelRadPerSec) {
    mode = ControlMode.WHEEL_RADIUS_CHARACTERIZATION;
    goalWheelRadiusCharacterizationAngularVel = angularVelRadPerSec;
  }

  public void endWheelRadiusCharacterization() {
    mode = ControlMode.TELEOP;
    goalWheelRadiusCharacterizationAngularVel = 0.0;
  }

  private ChassisSpeeds applyAccelLimitation(
      final ChassisSpeeds currentVel, final ChassisSpeeds goalVel) {
    var currentTranslationVel =
        new Translation2d(currentVel.vxMetersPerSecond, currentVel.vyMetersPerSecond);

    var goalTranslationVel =
        new Translation2d(goalVel.vxMetersPerSecond, goalVel.vyMetersPerSecond);

    var deltaVel = goalTranslationVel.minus(currentTranslationVel);

    if (EqualsUtil.epsilonEquals(deltaVel.getNorm(), 0.0)) {
      return new ChassisSpeeds(0.0, 0.0, goalVel.omegaRadiansPerSecond);
    }

    var customMaxTiltAccelScaleVal = customMaxTiltAccelScale.get();
    Logger.recordOutput("Swerve/customMaxTiltAccelScale", customMaxTiltAccelScaleVal);

    var maxTiltLimitedDeltaVelX =
        maxTiltAccelXMeterPerSec2.get() * customMaxTiltAccelScaleVal * Config.LOOP_PERIOD_SEC;
    var maxTiltLimitedDeltaVelY =
        maxTiltAccelYMeterPerSec2.get() * customMaxTiltAccelScaleVal * Config.LOOP_PERIOD_SEC;

    deltaVel =
        new Translation2d(
            MathUtil.clamp(deltaVel.getX(), -maxTiltLimitedDeltaVelX, maxTiltLimitedDeltaVelX),
            MathUtil.clamp(deltaVel.getY(), -maxTiltLimitedDeltaVelY, maxTiltLimitedDeltaVelY));

    var maxSkidLimitedDeltaVel = maxSkidAccelMeterPerSec2.get() * Config.LOOP_PERIOD_SEC;

    deltaVel =
        new Translation2d(
            MathUtil.clamp(deltaVel.getNorm(), -maxSkidLimitedDeltaVel, maxSkidLimitedDeltaVel),
            deltaVel.toRotation2d());

    var noForwardLimitedGoalVel = currentTranslationVel.plus(deltaVel);

    var finalGoalVel = noForwardLimitedGoalVel;

    if (noForwardLimitedGoalVel.getNorm() > currentTranslationVel.getNorm()) {
      var forwardLimitedDeltaVelVal =
          maxForwardAccelMeterPerSec2.get()
              * (1.0
                  - MathUtil.clamp(
                      currentTranslationVel.getNorm()
                          / SwerveConfig.MAX_TRANSLATION_VEL_METER_PER_SEC,
                      0.0,
                      1.0))
              * Config.LOOP_PERIOD_SEC;

      finalGoalVel =
          new Translation2d(
              currentTranslationVel.getNorm()
                  + Math.min(
                      noForwardLimitedGoalVel.getNorm() - currentTranslationVel.getNorm(),
                      forwardLimitedDeltaVelVal),
              finalGoalVel.toRotation2d());
    }

    return new ChassisSpeeds(
        finalGoalVel.getX(), finalGoalVel.getY(), goalVel.omegaRadiansPerSecond);
  }

  public Command getDriveSysidCmd(int moduleIndex) {
    var sysidRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null, // Use default ramp rate (1 V/s)
                Volts.of(4), // Reduce dynamic voltage to 4 to prevent brownout
                null, // Use default timeout (10 s)
                state -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                voltage -> {
                  var voltageVolt = voltage.in(Volts);
                  modules[moduleIndex].setDriveVoltage(voltageVolt);

                  if (Config.MODE == Config.Mode.SIM) {
                    SignalLogger.writeDouble("voltage", voltageVolt);
                    SignalLogger.writeDouble(
                        "positionRotation",
                        Units.radiansToRotations(modules[moduleIndex].getDrivePositionRad()));
                    SignalLogger.writeDouble(
                        "velocityRotationPerSec",
                        Units.radiansToRotations(modules[moduleIndex].getDriveVelRadPerSec()));
                  }
                },
                null,
                this));

    return Commands.sequence(
            Commands.runOnce(
                () -> {
                  mode = ControlMode.SYSID;
                  System.out.println("Module " + moduleIndex + " Drive Sysid start");
                  SignalLogger.start();
                  System.out.println("CTRE Signal Logger start");
                }),
            Commands.waitSeconds(0.5),
            Commands.print("Quasistatic forward start"),
            sysidRoutine.quasistatic(SysIdRoutine.Direction.kForward),
            Commands.print("Quasistatic forward done"),
            Commands.print("Quasistatic reverse start"),
            sysidRoutine.quasistatic(SysIdRoutine.Direction.kReverse),
            Commands.print("Quasistatic reverse done"),
            Commands.print("Dynamic forward start"),
            sysidRoutine.dynamic(SysIdRoutine.Direction.kForward),
            Commands.print("Dynamic forward done"),
            Commands.print("Dynamic reverse start"),
            sysidRoutine.dynamic(SysIdRoutine.Direction.kReverse),
            Commands.print("Dynamic reverse done"),
            Commands.waitSeconds(0.5))
        .finallyDo(
            interrupted -> {
              SignalLogger.stop();
              System.out.println("CTRE Signal Logger stop");

              mode = ControlMode.TELEOP;
              System.out.println(
                  "Module "
                      + moduleIndex
                      + " Drive Sysid "
                      + (interrupted ? "interrupted" : "done"));
            });
  }
}
