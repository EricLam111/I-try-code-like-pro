package com.nextinnovation.team8214.subsystem.swerve.controller;

import com.nextinnovation.team8214.Config;
import com.nextinnovation.team8214.Odometry;
import com.nextinnovation.team8214.subsystem.swerve.SwerveConfig;
import com.nextinnovation.team8214.util.LoggedTunableNumber;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class HeadingController {
  private static final double MAX_ALLOWABLE_ANGULAR_VEL_RAD_PER_SEC =
      SwerveConfig.MAX_ANGULAR_VEL_RAD_PER_SEC * 0.8;

  private static final LoggedTunableNumber kp =
      new LoggedTunableNumber(
          Config.LiveDebugGroup.SWERVE.toString(), "Swerve/HeadingController/kP", 4.0);
  private static final LoggedTunableNumber kd =
      new LoggedTunableNumber(
          Config.LiveDebugGroup.SWERVE.toString(), "Swerve/HeadingController/kD", 0.0);
  private static final LoggedTunableNumber maxVelocityRadPerSec =
      new LoggedTunableNumber(
          Config.LiveDebugGroup.SWERVE.toString(),
          "Swerve/HeadingController/MaxVelocityRadPerSec",
          MAX_ALLOWABLE_ANGULAR_VEL_RAD_PER_SEC);
  private static final LoggedTunableNumber maxAccelerationRadPerSec2 =
      new LoggedTunableNumber(
          Config.LiveDebugGroup.SWERVE.toString(),
          "Swerve/HeadingController/MaxAccelerationRadPerSec2",
          MAX_ALLOWABLE_ANGULAR_VEL_RAD_PER_SEC / 0.3);
  private static final LoggedTunableNumber toleranceDegree =
      new LoggedTunableNumber(
          Config.LiveDebugGroup.SWERVE.toString(), "Swerve/HeadingController/ToleranceDegree", 3.0);

  private final ProfiledPIDController pid;

  private final Supplier<Rotation2d> goadHeadingSupplier;

  public HeadingController(Supplier<Rotation2d> goalHeadingSupplier) {
    pid =
        new ProfiledPIDController(
            kp.get(),
            0.0,
            kd.get(),
            new TrapezoidProfile.Constraints(0.0, 0.0),
            Config.LOOP_PERIOD_SEC);
    pid.enableContinuousInput(-Math.PI, Math.PI);
    pid.setTolerance(Units.degreesToRadians(toleranceDegree.get()));

    this.goadHeadingSupplier = goalHeadingSupplier;

    pid.reset(
        Odometry.getInstance().getEstimatedPose().getRotation().getRadians(),
        Odometry.getInstance().getFieldCentricVel().dtheta);
  }

  public double update() {
    pid.setPID(kp.get(), 0.0, kd.get());
    pid.setTolerance(Units.degreesToRadians(toleranceDegree.get()));
    pid.setConstraints(
        new TrapezoidProfile.Constraints(
            maxVelocityRadPerSec.get(), maxAccelerationRadPerSec2.get()));

    var goalHeading = goadHeadingSupplier.get();
    Logger.recordOutput("Swerve/HeadingController/GoalHeading", goalHeading);
    var output =
        pid.calculate(
            Odometry.getInstance().getEstimatedPose().getRotation().getRadians(),
            goalHeading.getRadians());

    Logger.recordOutput("Swerve/HeadingController/Error", pid.getPositionError());

    return output;
  }

  @AutoLogOutput(key = "Swerve/HeadingController/AtGoal")
  public boolean atGoal() {
    return pid.atGoal();
  }
}
