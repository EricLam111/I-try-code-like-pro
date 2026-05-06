package com.nextinnovation.team8214;

import com.nextinnovation.team8214.subsystem.swerve.Swerve;
import com.nextinnovation.team8214.util.AllianceFlipUtil;
import com.nextinnovation.team8214.util.oi.CommandVader4ProController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

public class RobotContainer {
  private final Swerve swerve;

  private final Odometry odometry = Odometry.getInstance();

  private final CommandVader4ProController driver =
      new CommandVader4ProController(Ports.Joystick.DRIVER);

  RobotContainer() {
    swerve = new Swerve();

    configButtonBindings();
  }

  private void configButtonBindings() {
    CommandScheduler.getInstance().getActiveButtonLoop().clear();

    swerve.setDefaultCommand(
        swerve
            .run(
                () ->
                    swerve.setTeleopInput(
                        -driver.getLeftY(),
                        -driver.getLeftX(),
                        -driver.getRightX(),
                        driver.rightBumper().getAsBoolean()))
            .withName("Swerve Teleop"));

    driver
        .select()
        .onTrue(
            Commands.runOnce(
                    () -> {
                      swerve.disableTeleopControllerHeadingMaintainer();
                      odometry.resetPose(
                          new Pose2d(
                              odometry.getEstimatedPose().getTranslation(),
                              AllianceFlipUtil.apply(new Rotation2d())));
                    })
                .ignoringDisable(true)
                .withName("Swerve Home Gyro"));
  }

  public Command getAutoCmd() {
    return Commands.runOnce(
        () -> {
          var trajectory = TrajectoryLoader.getInstance().getTrajectorySet().test.get();
          odometry.resetPose(trajectory.getInitialPose(false).get());
          swerve.setTrajectory(trajectory);
        },
        swerve);
  }
}
