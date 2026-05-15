package com.nextinnovation.team8214.subsystem.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

class ApriltagVisionConfig {
  static final Pose3d INTAKE_LEFT_IN_ROBOT =
      new Pose3d(
          0.309279,
          0.286145,
          0.206800,
          new Rotation3d(0.0, Units.degreesToRadians(-15.0), Units.degreesToRadians(-40.0)));
  static final Pose3d INTAKE_RIGHT_IN_ROBOT =
      new Pose3d(
          0.309279,
          -0.286145,
          0.206800,
          new Rotation3d(0.0, Units.degreesToRadians(-15.0), Units.degreesToRadians(40.0)));
  static final Pose3d REEF_BACK_LEFT_IN_ROBOT =
      new Pose3d(
          -0.319882,
          0.293060,
          0.221800,
          new Rotation3d(0.0, Units.degreesToRadians(-15.0), Units.degreesToRadians(-155.0)));
  static final Pose3d REEF_BACK_RIGHT_IN_ROBOT =
      new Pose3d(
          -0.319882,
          -0.293060,
          0.221800,
          new Rotation3d(0.0, Units.degreesToRadians(-15.0), Units.degreesToRadians(155.0)));
  static final double FIELD_BORDER_THRESHOLD_METER = 0.5;
  static final double ROBOT_POSE_Z_THRESHOLD_METER = 0.5;
}
