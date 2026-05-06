package com.nextinnovation.team8214;

import com.nextinnovation.team8214.util.BooleanChooser;
import com.nextinnovation.team8214.util.LoggedTunableNumber;
import java.util.function.BooleanSupplier;
import lombok.RequiredArgsConstructor;

public final class Config {
  public static final boolean ENABLE_SINGLE_TAG_POSE = true;

  public static final double LOOP_PERIOD_SEC = 0.02;

  public static final boolean IS_LIVE_DEBUG = false;

  @RequiredArgsConstructor
  public enum LiveDebugGroup {
    GLOBAL("Global", new BooleanChooser("LiveDebugGroupChooser/Global")),
    SWERVE("Swerve", new BooleanChooser("LiveDebugGroupChooser/Swerve")),
    ODOMETRY("Odometry", new BooleanChooser("LiveDebugGroupChooser/Odometry")),
    TURRET("Turret", new BooleanChooser("LiveDebugGroupChooser/Turret")),
    ;

    private final String name;
    private final BooleanSupplier groupActiveSupplier;

    public static void updateGroupActive() {
      LoggedTunableNumber.setGroupActive(
          GLOBAL.toString(), GLOBAL.groupActiveSupplier.getAsBoolean());
    }

    @Override
    public String toString() {
      return name;
    }
  }

  public static final Mode MODE = Mode.SIM;

  public enum Mode {
    REAL,
    SIM,
    REPLAY;
  }

  /** Checks whether the correct mode is selected when deploying. */
  public static class CheckDeploy {
    public static void main(String... args) {
      if (MODE != Mode.REAL) {
        System.err.println("Cannot deploy, invalid robot selected: " + MODE);
        System.exit(1);
      }
    }
  }

  /** Checks whether the correct mode is selected when simulating. */
  public static class CheckSim {
    public static void main(String... args) {
      if (MODE == Mode.REAL) {
        System.err.println("Cannot sim, invalid robot selected: " + MODE);
        System.exit(1);
      }
    }
  }

  /** Checks that the default robot is selected and tuning mode is disabled. */
  public static class CheckPullRequest {
    public static void main(String... args) {
      if (MODE != Mode.REAL || IS_LIVE_DEBUG) {
        System.err.println("Do not merge, non-default constants are configured.");
        System.exit(1);
      }
    }
  }
}
