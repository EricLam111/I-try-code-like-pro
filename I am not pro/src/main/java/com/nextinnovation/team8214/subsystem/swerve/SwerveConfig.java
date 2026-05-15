package com.nextinnovation.team8214.subsystem.swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.nextinnovation.team8214.Config;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;

public class SwerveConfig {
  static final String FL_MODULE_NAME = "FL";
  static final String BL_MODULE_NAME = "BL";
  static final String BR_MODULE_NAME = "BR";
  static final String FR_MODULE_NAME = "FR";

  static final double ODOMETRY_FREQUENCY_HZ = 250.0;

  public static final double WHEELBASE_LENGTH_METER = 0.2967 * 2.0;
  public static final double WHEELBASE_WIDTH_METER = 0.2967 * 2.0;
  public static final double WHEELBASE_DIAGONAL_METER =
      Math.hypot(WHEELBASE_LENGTH_METER, WHEELBASE_WIDTH_METER);

  public static final double MAX_TRANSLATION_VEL_METER_PER_SEC = 5.12064; // SDS MK5n R2
  public static final double MAX_ANGULAR_VEL_RAD_PER_SEC =
      MAX_TRANSLATION_VEL_METER_PER_SEC / (WHEELBASE_DIAGONAL_METER / 2.0);

  public static final SwerveDriveKinematics SWERVE_KINEMATICS =
      new SwerveDriveKinematics(
          new Translation2d(WHEELBASE_LENGTH_METER / 2.0, WHEELBASE_WIDTH_METER / 2.0),
          new Translation2d(-WHEELBASE_LENGTH_METER / 2.0, WHEELBASE_WIDTH_METER / 2.0),
          new Translation2d(-WHEELBASE_LENGTH_METER / 2.0, -WHEELBASE_WIDTH_METER / 2.0),
          new Translation2d(WHEELBASE_LENGTH_METER / 2.0, -WHEELBASE_WIDTH_METER / 2.0));

  static final double WHEEL_RADIUS_METER = 0.0479;
  static final ModuleConfig FL_MODULE_CONFIG =
      new ModuleConfig(
          getX2iDriveTalonConfig(),
          getX2iSteerTalonNoEncoderConfig(),
          getCancoderConfig(-0.487548828125));
  static final ModuleConfig BL_MODULE_CONFIG =
      new ModuleConfig(
          getX2iDriveTalonConfig(),
          getX2iSteerTalonNoEncoderConfig(),
          getCancoderConfig(-0.359619140625));
  static final ModuleConfig BR_MODULE_CONFIG =
      new ModuleConfig(
          getX2iDriveTalonConfig(),
          getX2iSteerTalonNoEncoderConfig(),
          getCancoderConfig(0.060302734375));
  static final ModuleConfig FR_MODULE_CONFIG =
      new ModuleConfig(
          getX2iDriveTalonConfig(),
          getX2iSteerTalonNoEncoderConfig(),
          getCancoderConfig(-0.364990234375));

  /// WCP X2i Reductions
  /// https://docs.wcproducts.com/wcp-swerve-x2/general-info/ratio-options
  /// - X1T10: (54.0 / 10.0) * (18.0 / 38.0) * (45.0 / 15.0) = 7.67
  /// - X1T11: (54.0 / 11.0) * (18.0 / 38.0) * (45.0 / 15.0) = 6.98
  /// - X1T12: (54.0 / 12.0) * (18.0 / 38.0) * (45.0 / 15.0) = 6.39
  /// - X2T10: (54.0 / 10.0) * (16.0 / 38.0) * (45.0 / 15.0) = 6.82
  /// - X2T11: (54.0 / 11.0) * (16.0 / 38.0) * (45.0 / 15.0) = 6.20
  /// - X2T12: (54.0 / 12.0) * (16.0 / 38.0) * (45.0 / 15.0) = 5.68
  /// - X3T10: (54.0 / 10.0) * (16.0 / 40.0) * (45.0 / 15.0) = 6.48
  /// - X3T11: (54.0 / 11.0) * (16.0 / 40.0) * (45.0 / 15.0) = 5.89
  /// - X3T12: (54.0 / 12.0) * (16.0 / 40.0) * (45.0 / 15.0) = 5.40
  /// - X4T10: (54.0 / 10.0) * (14.0 / 40.0) * (45.0 / 15.0) = 5.67
  /// - X4T11: (54.0 / 11.0) * (14.0 / 40.0) * (45.0 / 15.0) = 5.15
  /// - X4T12: (54.0 / 12.0) * (14.0 / 40.0) * (45.0 / 15.0) = 4.73
  /// - TURN: (88.0 / 16.0) * (22.0 / 27.0) * (27.0 / 10.0) = 12.1
  ///
  /// SDS MK5n Reductions
  /// https://www.swervedrivespecialties.com/products/mk5n-swerve-module
  /// - R1: (54.0 / 12.0) * (25.0 / 32.0) * (30.0 / 15.0)
  /// - R2: (54.0 / 14.0) * (25.0 / 32.0) * (30.0 / 15.0)
  /// - R3: (54.0 / 16.0) * (25.0 / 32.0) * (30.0 / 15.0)
  ///
  static final double DRIVE_REDUCTION = (54.0 / 14.0) * (25.0 / 32.0) * (30.0 / 15.0);
  static final double STEER_REDUCTION = (88.0 / 16.0) * (22.0 / 27.0) * (27.0 / 10.0);

  static final double DRIVE_FF_KT =
      DCMotor.getKrakenX60Foc(1).withReduction(DRIVE_REDUCTION).KtNMPerAmp;

  record ModuleConfig(
      TalonFXConfiguration driveTalonConfig,
      TalonFXConfiguration steerTalonConfig,
      CANcoderConfiguration cancoderConfig) {}

  static Gains getDriveGains() {
    // Kv unit should be rotations/s
    return switch (Config.MODE) {
      case REAL -> new Gains(2.0, 0.0, 0.124 * DRIVE_REDUCTION, 0.0);
      case SIM, REPLAY -> new Gains(0.3, 0.0, 0.124 * DRIVE_REDUCTION, 0.0);
    };
  }

  static Gains getSteerGains() {
    return switch (Config.MODE) {
      case REAL -> new Gains(100.0, 0.5, 0.0, 0.1);
      case SIM, REPLAY -> new Gains(10.0, 0.0, 0.0, 0.0);
    };
  }

  record Gains(double kp, double kd, double kv, double ks) {}

  static TalonFXConfiguration getX2iDriveTalonConfig() {
    var config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    var gains = getDriveGains();
    config.Slot0 =
        new Slot0Configs()
            .withKP(gains.kp())
            .withKD(gains.kd())
            .withKV(gains.kv())
            .withKS(gains.ks());

    config.CurrentLimits.StatorCurrentLimit = 200.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.Feedback.SensorToMechanismRatio = DRIVE_REDUCTION;

    return config;
  }

  static TalonFXConfiguration getX2iSteerTalonNoEncoderConfig() {
    var config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    var gains = getSteerGains();
    config.Slot0 =
        new Slot0Configs()
            .withKP(gains.kp())
            .withKD(gains.kd())
            .withKS(gains.ks())
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

    config.CurrentLimits.StatorCurrentLimit = 60.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.Feedback.RotorToSensorRatio = STEER_REDUCTION;

    config.ClosedLoopGeneral.ContinuousWrap = true;

    return config;
  }

  private static CANcoderConfiguration getCancoderConfig(double magnetOffset) {
    var config = new CANcoderConfiguration();
    config.MagnetSensor.MagnetOffset = magnetOffset;

    return config;
  }

  private static final double WHEELBASE_HEIGHT_METER = .07;
  private static final Rotation3d WHEELBASE_ROTATION3D =
      new Rotation3d(new Quaternion(.5, .5, .5, .5));
  public static final Transform3d FL_ZEROED_TF =
      new Transform3d(
          -WHEELBASE_WIDTH_METER / 2.0,
          -WHEELBASE_LENGTH_METER / 2.0,
          WHEELBASE_HEIGHT_METER,
          WHEELBASE_ROTATION3D);
  public static final Transform3d BL_ZEROED_TF =
      new Transform3d(
          -WHEELBASE_WIDTH_METER / 2.0,
          WHEELBASE_LENGTH_METER / 2.0,
          WHEELBASE_HEIGHT_METER,
          WHEELBASE_ROTATION3D);
  public static final Transform3d BR_ZEROED_TF =
      new Transform3d(
          WHEELBASE_WIDTH_METER / 2.0,
          WHEELBASE_LENGTH_METER / 2.0,
          WHEELBASE_HEIGHT_METER,
          WHEELBASE_ROTATION3D);
  public static final Transform3d FR_ZEROED_TF =
      new Transform3d(
          WHEELBASE_WIDTH_METER / 2.0,
          -WHEELBASE_LENGTH_METER / 2.0,
          WHEELBASE_HEIGHT_METER,
          WHEELBASE_ROTATION3D);
}
