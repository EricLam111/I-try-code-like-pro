package com.nextinnovation.team8214.subsystem.swerve.controller;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.nextinnovation.team8214.Odometry;
import com.nextinnovation.team8214.util.EqualsUtil;
import com.nextinnovation.team8214.util.GeomUtil;
import com.nextinnovation.team8214.util.LoggedTunableNumber;
import com.nextinnovation.team8214.util.superautopilot.SAPProfile;
import com.nextinnovation.team8214.util.superautopilot.SuperAutopilot;
import com.therekrab.autopilot.APTarget;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import java.util.Optional;
import java.util.function.Supplier;
import lombok.experimental.ExtensionMethod;

@ExtensionMethod({GeomUtil.class})
public class DriveToPointController {
  public static class DriveToPointControllerConfig {
    private final LoggedTunableNumber xToleranceMeterTunable;
    private final LoggedTunableNumber yToleranceMeterTunable;
    private final LoggedTunableNumber maxTranslationVelMeterPerSecTunable;
    private final LoggedTunableNumber maxTranslationStartAccelMeterPerSec2Tunable;
    private final LoggedTunableNumber maxTranslationEndJerkMeterPerSec3Tunable;
    private final LoggedTunableNumber beelineFreeDistanceMeterTunable;
    private final LoggedTunableNumber autopilotDistanceMeterTunable;
    private final LoggedTunableNumber transitionShiftingXMeterTunable;
    private final LoggedTunableNumber rotationKpTunable;
    private final LoggedTunableNumber rotationKdTunable;
    private final LoggedTunableNumber rotationToleranceDegreeTunable;
    private final LoggedTunableNumber maxRotationVelDegreePerSecTunable;

    protected double xToleranceMeter;
    protected double yToleranceMeter;
    protected double maxTranslationVelMeterPerSec;
    protected double maxTranslationStartAccelMeterPerSec2;
    protected double maxTranslationEndJerkMeterPerSec3;
    protected double beelineFreeDistanceMeter;
    protected double autopilotDistanceMeter;
    protected double transitionShiftingXMeter;
    protected double rotationKp;
    protected double rotationKd;
    protected double rotationToleranceDegree;
    protected double maxRotationVelDegreePerSec;

    public DriveToPointControllerConfig(
        String liveDebugGroup,
        String key,
        double xToleranceMeter,
        double yToleranceMeter,
        double maxTranslationVelMeterPerSec,
        double maxTranslationStartAccelMeterPerSec2,
        double maxTranslationEndJerkMeterPerSec3,
        double beelineFreeDistanceMeter,
        double autopilotDistanceMeter,
        double transitionShiftingXMeter,
        double rotationKp,
        double rotationKd,
        double rotationToleranceDegree,
        double maxRotationVelDegreePerSec) {
      xToleranceMeterTunable =
          new LoggedTunableNumber(liveDebugGroup, key + "/XToleranceMeter", xToleranceMeter);
      yToleranceMeterTunable =
          new LoggedTunableNumber(liveDebugGroup, key + "/YToleranceMeter", yToleranceMeter);
      maxTranslationVelMeterPerSecTunable =
          new LoggedTunableNumber(
              liveDebugGroup, key + "/MaxTranslationVelMeterPerSec", maxTranslationVelMeterPerSec);
      maxTranslationStartAccelMeterPerSec2Tunable =
          new LoggedTunableNumber(
              liveDebugGroup,
              key + "/MaxTranslationAccelMeterPerSec2",
              maxTranslationStartAccelMeterPerSec2);
      maxTranslationEndJerkMeterPerSec3Tunable =
          new LoggedTunableNumber(
              liveDebugGroup,
              key + "/MaxTranslationEndJerkMeterPerSec3",
              maxTranslationEndJerkMeterPerSec3);
      beelineFreeDistanceMeterTunable =
          new LoggedTunableNumber(
              liveDebugGroup, key + "/BeelineDistanceMeter", beelineFreeDistanceMeter);
      autopilotDistanceMeterTunable =
          new LoggedTunableNumber(
              liveDebugGroup, key + "/AutopilotDistanceMeter", autopilotDistanceMeter);
      transitionShiftingXMeterTunable =
          new LoggedTunableNumber(
              liveDebugGroup, key + "/TransitionShiftingXMeter", transitionShiftingXMeter);
      rotationKpTunable = new LoggedTunableNumber(liveDebugGroup, key + "/RotationKp", rotationKp);
      rotationKdTunable = new LoggedTunableNumber(liveDebugGroup, key + "/RotationKd", rotationKd);
      rotationToleranceDegreeTunable =
          new LoggedTunableNumber(
              liveDebugGroup, key + "/RotationToleranceDegree", rotationToleranceDegree);
      maxRotationVelDegreePerSecTunable =
          new LoggedTunableNumber(
              liveDebugGroup, key + "/MaxRotationDegreePerSec", maxRotationVelDegreePerSec);

      flush();
    }

    public void flush() {
      xToleranceMeter = xToleranceMeterTunable.get();
      yToleranceMeter = yToleranceMeterTunable.get();
      maxTranslationVelMeterPerSec = maxTranslationVelMeterPerSecTunable.get();
      maxTranslationStartAccelMeterPerSec2 = maxTranslationStartAccelMeterPerSec2Tunable.get();
      maxTranslationEndJerkMeterPerSec3 = maxTranslationEndJerkMeterPerSec3Tunable.get();
      beelineFreeDistanceMeter = beelineFreeDistanceMeterTunable.get();
      autopilotDistanceMeter = autopilotDistanceMeterTunable.get();
      transitionShiftingXMeter = transitionShiftingXMeterTunable.get();
      rotationKp = rotationKpTunable.get();
      rotationKd = rotationKdTunable.get();
      rotationToleranceDegree = rotationToleranceDegreeTunable.get();
      maxRotationVelDegreePerSec = maxRotationVelDegreePerSecTunable.get();
    }

    public SAPProfile toSAPProfile() {
      return new SAPProfile(
          maxTranslationVelMeterPerSec,
          maxTranslationStartAccelMeterPerSec2,
          maxTranslationEndJerkMeterPerSec3,
          Meters.of(xToleranceMeter),
          Meters.of(yToleranceMeter),
          Degrees.of(rotationToleranceDegree),
          Meters.of(beelineFreeDistanceMeter),
          Meters.of(autopilotDistanceMeter),
          Meters.of(transitionShiftingXMeter));
    }
  }

  private final DriveToPointControllerConfig config;

  private SuperAutopilot superAutopilot;
  private final PIDController rotationController = new PIDController(0.0, 0.0, 0.0);
  private final Supplier<Optional<Pose2d>> goalPoseSupplier;

  public DriveToPointController(
      DriveToPointControllerConfig config, Supplier<Optional<Pose2d>> goalPoseSupplier) {
    this.config = config;
    this.goalPoseSupplier = goalPoseSupplier;
    this.superAutopilot = new SuperAutopilot(config.toSAPProfile());

    rotationController.enableContinuousInput(-Math.PI, Math.PI);
  }

  public ChassisSpeeds update() {
    config.flush();

    rotationController.setP(config.rotationKp);
    rotationController.setD(config.rotationKd);
    rotationController.setTolerance(config.rotationToleranceDegree);

    var currentPose = getCurrentPose();

    var odometry = Odometry.getInstance();
    var vel = odometry.getRobotCentricVel();

    var shiftedGoalPose = getShiftedGoalPose();
    var entryAngle = getEntryAngle();

    if (shiftedGoalPose.isEmpty() || entryAngle.isEmpty()) {
      return new ChassisSpeeds();
    }

    superAutopilot = new SuperAutopilot(config.toSAPProfile());
    var superAutopilotOutput =
        superAutopilot.calculate(
            currentPose, vel, new APTarget(shiftedGoalPose.get()).withEntryAngle(entryAngle.get()));

    var rotationVel =
        MathUtil.clamp(
            rotationController.calculate(
                MathUtil.angleModulus(currentPose.getRotation().getRadians()),
                MathUtil.angleModulus(superAutopilotOutput.targetAngle().getRadians())),
            Units.degreesToRadians(-config.maxRotationVelDegreePerSec),
            Units.degreesToRadians(config.maxRotationVelDegreePerSec));

    if (hasDone()) {
      return new ChassisSpeeds();
    } else {
      return ChassisSpeeds.fromFieldRelativeSpeeds(
          superAutopilotOutput.vx().in(MetersPerSecond),
          superAutopilotOutput.vy().in(MetersPerSecond),
          rotationVel,
          currentPose.getRotation());
    }
  }

  protected Pose2d getCurrentPose() {
    return Odometry.getInstance().getEstimatedPose();
  }

  protected Optional<Pose2d> getShiftedGoalPose() {
    return getGoalPose();
  }

  protected Optional<Rotation2d> getEntryAngle() {
    var goalPose = getGoalPose();

    return goalPose.map(Pose2d::getRotation);
  }

  protected Optional<Pose2d> getGoalPose() {
    return goalPoseSupplier.get();
  }

  public boolean hasDone() {
    var currentPose = getCurrentPose();
    var goalPose = getGoalPose();
    var entryAngle = getEntryAngle();

    if (goalPose.isEmpty() || entryAngle.isEmpty()) {
      return false;
    }

    return superAutopilot.atPos(currentPose, goalPose.get(), entryAngle.get())
        && hasHeadingAtGoal();
  }

  public boolean hasHeadingAtGoal() {
    return rotationController.atSetpoint();
  }

  public boolean hasDistanceWithin(double toleranceMeter) {
    var goalPose = getGoalPose();

    return goalPose
        .filter(
            pose2d ->
                getCurrentPose().getTranslation().getDistance(pose2d.getTranslation())
                    <= toleranceMeter)
        .isPresent();
  }

  public boolean hasHeadingWithin(double toleranceDegree) {
    var goalPose = getGoalPose();

    return goalPose
        .filter(
            pose2d ->
                EqualsUtil.GeomExtensions.epsilonEquals(
                    getCurrentPose().getRotation(),
                    pose2d.getRotation(),
                    Units.degreesToRadians(toleranceDegree),
                    true))
        .isPresent();
  }
}
