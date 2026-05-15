package com.nextinnovation.team8214.subsystem.swerve.controller;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import com.nextinnovation.team8214.Config;
import com.nextinnovation.team8214.Odometry;
import com.nextinnovation.team8214.util.GeomUtil;
import com.nextinnovation.team8214.util.LoggedTunableNumber;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.IntStream;
import lombok.Getter;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.junction.Logger;

@ExtensionMethod({GeomUtil.class})
public class TrajectoryController {
  private static final LoggedTunableNumber translationKp =
      new LoggedTunableNumber(
          Config.LiveDebugGroup.SWERVE.toString(),
          "Swerve/TrajectoryController/TranslationKp",
          8.0);
  private static final LoggedTunableNumber translationKd =
      new LoggedTunableNumber(
          Config.LiveDebugGroup.SWERVE.toString(),
          "Swerve/TrajectoryController/TranslationKd",
          0.0);
  private static final LoggedTunableNumber rotationKp =
      new LoggedTunableNumber(
          Config.LiveDebugGroup.SWERVE.toString(), "Swerve/TrajectoryController/RotationKp", 5.0);
  private static final LoggedTunableNumber rotationKd =
      new LoggedTunableNumber(
          Config.LiveDebugGroup.SWERVE.toString(), "Swerve/TrajectoryController/RotationKd", 0.0);

  private final Trajectory<SwerveSample> trajectory;
  private final PIDController xController;
  private final PIDController yController;
  private final PIDController rotationController;
  private final Timer timer = new Timer();

  @Getter
  private List<Vector<N2>> moduleForces =
      IntStream.range(0, 4).boxed().map(i -> VecBuilder.fill(0, 0)).toList();

  private SwerveSample setpoint = null;
  private boolean hasStart = false;

  public TrajectoryController(Trajectory<SwerveSample> trajectory) {
    this.trajectory = trajectory;

    xController = new PIDController(translationKp.get(), 0.0, translationKd.get());
    yController = new PIDController(translationKp.get(), 0.0, translationKd.get());
    rotationController = new PIDController(rotationKp.get(), 0.0, rotationKd.get());
    rotationController.enableContinuousInput(-Math.PI, Math.PI);

    // Log poses
    Logger.recordOutput("Swerve/TrajectoryController/TrajectoryPoses", trajectory.getPoses());
  }

  public ChassisSpeeds update() {
    if (setpoint != null) {
      Logger.recordOutput("Swerve/TrajectoryController/SetpointPose", setpoint.getPose());
      Logger.recordOutput("Swerve/TrajectoryController/SetpointVel/Vx", setpoint.vx);
      Logger.recordOutput("Swerve/TrajectoryController/SetpointVel/Vy", setpoint.vy);
      Logger.recordOutput("Swerve/TrajectoryController/SetpointVel/Rotation", setpoint.omega);
    }

    if (!hasStart) {
      hasStart = true;
      setpoint = trajectory.getInitialSample(false).get();
      timer.start();
    } else if (!hasDone()) {
      setpoint = trajectory.sampleAt(timer.get(), false).get();
    } else {
      setpoint = trajectory.getInitialSample(false).get();
      return new ChassisSpeeds(0.0, 0.0, 0.0);
    }

    var trajectoryVel =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            setpoint.vx, setpoint.vy, setpoint.omega, setpoint.getPose().getRotation());

    Odometry.getInstance().addTrajectoryVel(trajectoryVel.toTwist2d());

    var currentPose = Odometry.getInstance().getEstimatedPose();
    var setpointPose = setpoint.getPose();

    var xFeedback = xController.calculate(currentPose.getX(), setpointPose.getX());
    var yFeedback = yController.calculate(currentPose.getY(), setpointPose.getY());
    var rotationFeedback =
        rotationController.calculate(
            MathUtil.angleModulus(currentPose.getRotation().getRadians()),
            MathUtil.angleModulus(setpointPose.getRotation().getRadians()));

    var setpointHeading = setpointPose.getRotation();
    var moduleForcesX = setpoint.moduleForcesX();
    var moduleForcesY = setpoint.moduleForcesY();

    // [FL, FR, BL, BR] -> [FL, BL, BR, FR]
    moduleForces = new ArrayList<>(4);
    moduleForces.add(fixModuleForce(moduleForcesX[0], moduleForcesY[0], setpointHeading));
    moduleForces.add(fixModuleForce(moduleForcesX[2], moduleForcesY[2], setpointHeading));
    moduleForces.add(fixModuleForce(moduleForcesX[3], moduleForcesY[3], setpointHeading));
    moduleForces.add(fixModuleForce(moduleForcesX[1], moduleForcesY[1], setpointHeading));

    Logger.recordOutput(
        "Swerve/TrajectoryController/TranslationError",
        currentPose.getTranslation().getDistance(setpointPose.getTranslation()));
    Logger.recordOutput(
        "Swerve/TrajectoryController/RotationError",
        currentPose.getRotation().minus(setpointPose.getRotation()));

    return ChassisSpeeds.fromFieldRelativeSpeeds(
        setpoint.vx + xFeedback,
        setpoint.vy + yFeedback,
        setpoint.omega + rotationFeedback,
        currentPose.getRotation());
  }

  public boolean hasDone() {
    return timer.hasElapsed(trajectory.getTotalTime());
  }

  private static Vector<N2> fixModuleForce(double x, double y, Rotation2d headingInField) {
    return new Translation2d(x, y)
        .rotateBy(Rotation2d.fromRadians(headingInField.getRadians()).unaryMinus())
        .toVector();
  }
}
