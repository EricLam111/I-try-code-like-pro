package com.nextinnovation.team8214;

import com.ctre.phoenix6.SignalLogger;
import com.nextinnovation.team8214.util.LoggedTunableNumber;
import com.nextinnovation.team8214.util.VirtualSubsystem;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.util.HashMap;
import java.util.Map;
import java.util.function.BiConsumer;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
  private static final LoggedTunableNumber exampleLoggedTunableNumber =
      new LoggedTunableNumber(
          Config.LiveDebugGroup.GLOBAL.toString(), "exampleLoggedTunableNumber", 0.0);

  private final RobotContainer robotContainer;
  private Command autoCmd;

  private boolean autoMessagePrinted;
  private double autoStart;

  public Robot() {
    ctreLoggerInit();
    advantageKitLoggerInit();

    TrajectoryLoader.getInstance().lazyLoadTrajectorySet();

    RobotController.setBrownoutVoltage(6.0);
    robotContainer = new RobotContainer();

    WebServer.start(5800, Filesystem.getDeployDirectory().getPath());
  }

  private void ctreLoggerInit() {
    SignalLogger.enableAutoLogging(false);
    SignalLogger.stop();
  }

  private void advantageKitLoggerInit() {
    Logger.recordMetadata("IsLiveDebug", Boolean.toString(Config.IS_LIVE_DEBUG));
    Logger.recordMetadata("RuntimeType", getRuntimeType().toString());
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    switch (Config.MODE) {
      case REAL -> {
        Logger.addDataReceiver(new WPILOGWriter("/home/lvuser/logs"));
        if (Config.IS_LIVE_DEBUG) {
          Logger.addDataReceiver(new NT4Publisher());
        }
      }

      case SIM -> Logger.addDataReceiver(new NT4Publisher());

      case REPLAY -> {
        setUseTiming(false); // Run as fast as possible
        var logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
      }
    }

    Logger.start();

    Map<String, Integer> commandCounts = new HashMap<>();
    BiConsumer<Command, Boolean> logCommandFunction =
        (Command command, Boolean active) -> {
          String name = command.getName();
          int count = commandCounts.getOrDefault(name, 0) + (active ? 1 : -1);
          commandCounts.put(name, count);
          Logger.recordOutput(
              "CommandsUnique/" + name + "_" + Integer.toHexString(command.hashCode()), active);
          Logger.recordOutput("CommandsAll/" + name, count > 0);
        };
    CommandScheduler.getInstance()
        .onCommandInitialize((Command command) -> logCommandFunction.accept(command, true));
    CommandScheduler.getInstance()
        .onCommandFinish((Command command) -> logCommandFunction.accept(command, false));
    CommandScheduler.getInstance()
        .onCommandInterrupt((Command command) -> logCommandFunction.accept(command, false));
  }

  @Override
  public void robotPeriodic() {
    Threads.setCurrentThreadPriority(true, 99);

    if (autoCmd != null) {
      if (!autoCmd.isScheduled() && !autoMessagePrinted) {
        if (DriverStation.isAutonomousEnabled()) {
          System.out.printf(
              "*** Auto finished in %.2f secs ***%n", Timer.getFPGATimestamp() - autoStart);
        } else {
          System.out.printf(
              "*** Auto cancelled in %.2f secs ***%n", Timer.getFPGATimestamp() - autoStart);
        }
        autoMessagePrinted = true;
      }
    }

    if (Config.IS_LIVE_DEBUG && Config.MODE != Config.Mode.REPLAY) {
      Config.LiveDebugGroup.updateGroupActive();
    }

    var startTime = Timer.getFPGATimestamp();
    VirtualSubsystem.periodicAll();
    Logger.recordOutput(
        "Performance/VirtualSubsystem/periodicAll", Timer.getFPGATimestamp() - startTime);

    startTime = Timer.getFPGATimestamp();
    CommandScheduler.getInstance().run();
    Logger.recordOutput("Performance/CommandScheduler/run", Timer.getFPGATimestamp() - startTime);

    Odometry.getInstance().updateElasticPoses();

    Threads.setCurrentThreadPriority(true, 10);
  }

  @Override
  public void autonomousInit() {
    autoStart = Timer.getFPGATimestamp();
    autoMessagePrinted = false;
    autoCmd = robotContainer.getAutoCmd();

    if (autoCmd != null) {
      CommandScheduler.getInstance().schedule(autoCmd);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (autoCmd != null) {
      autoCmd.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
