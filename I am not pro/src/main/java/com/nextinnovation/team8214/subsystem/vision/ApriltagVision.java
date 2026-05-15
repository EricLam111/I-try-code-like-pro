package com.nextinnovation.team8214.subsystem.vision;

import com.nextinnovation.team8214.Config;
import com.nextinnovation.team8214.Field;
import com.nextinnovation.team8214.Odometry;
import com.nextinnovation.team8214.util.Alert;
import com.nextinnovation.team8214.util.GeomUtil;
import com.nextinnovation.team8214.util.LoggedTunableNumber;
import com.nextinnovation.team8214.util.VirtualSubsystem;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import java.util.*;
import lombok.RequiredArgsConstructor;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.junction.Logger;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

@ExtensionMethod({GeomUtil.class})
public class ApriltagVision extends VirtualSubsystem {
  private static final LoggedTunableNumber xyStdMeter =
      new LoggedTunableNumber(
          Config.LiveDebugGroup.ODOMETRY.toString(), "ApriltagVision/xyStdMeter", 0.01);
  private static final LoggedTunableNumber thetaStdDegree =
      new LoggedTunableNumber(
          Config.LiveDebugGroup.ODOMETRY.toString(), "ApriltagVision/thetaStdDegree", 5.0);
  private static final LoggedTunableNumber maxAllowedAmbiguity =
      new LoggedTunableNumber(
          Config.LiveDebugGroup.ODOMETRY.toString(), "ApriltagVision/maxAllowedAmbiguity", 0.3);

  private static SimCameraProperties KS1A293_1280_800() {
    var prop = new SimCameraProperties();
    prop.setCalibration(1280, 800, Rotation2d.fromDegrees(82.0));
    prop.setCalibError(0.37, 0.06);
    prop.setFPS(50.0);
    prop.setAvgLatencyMs(14);
    prop.setLatencyStdDevMs(5);
    return prop;
  }

  @RequiredArgsConstructor
  public enum CameraId {
    INTAKE_LEFT("IntakeLeft"),
    INTAKE_RIGHT("IntakeRight"),
    REEF_BACK_LEFT("ReefBackLeft"),
    REEF_BACK_RIGHT("ReefBackRight");

    private final String name;

    @Override
    public String toString() {
      return name;
    }
  }

  private final ApriltagVisionIO intakeLeftIo;
  private final ApriltagVisionIO intakeRightIo;
  private final ApriltagVisionIO reefBackLeftIo;
  private final ApriltagVisionIO reefBackRightIo;

  private final ApriltagVisionIOInputsAutoLogged intakeLeftInputs =
      new ApriltagVisionIOInputsAutoLogged();
  private final ApriltagVisionIOInputsAutoLogged intakeRightInputs =
      new ApriltagVisionIOInputsAutoLogged();
  private final ApriltagVisionIOInputsAutoLogged reefBackLeftInputs =
      new ApriltagVisionIOInputsAutoLogged();
  private final ApriltagVisionIOInputsAutoLogged reefBackRightInputs =
      new ApriltagVisionIOInputsAutoLogged();

  private final List<Odometry.VisionObservation> allGoodVisionObservations = new ArrayList<>();
  private final HashMap<CameraId, Map<Integer, Odometry.SingleTagVisionObservation>>
      allSingleTagVisionObservations = new HashMap<>();
  private final List<Pose3d> allGoodRobotInField = new ArrayList<>();
  private final List<Pose2d> allGoodRobotInField2d = new ArrayList<>();
  private final List<Pose3d> allBadRobotInField = new ArrayList<>();
  private final List<Pose3d> allUsedTagInField = new ArrayList<>();

  private final Alert intakeLeftOfflineAlert =
      new Alert(CameraId.INTAKE_LEFT + " offline!", Alert.AlertType.WARNING);
  private final Alert intakeRightOfflineAlert =
      new Alert(CameraId.INTAKE_RIGHT + " offline!", Alert.AlertType.WARNING);
  private final Alert reefBackLeftOfflineAlert =
      new Alert(CameraId.REEF_BACK_LEFT + " offline!", Alert.AlertType.WARNING);
  private final Alert reefBackRightOfflineAlert =
      new Alert(CameraId.REEF_BACK_RIGHT + " offline!", Alert.AlertType.WARNING);

  private VisionSystemSim visionSystemSim = null;

  public ApriltagVision() {
    switch (Config.MODE) {
      case REAL -> {
        intakeLeftIo = new ApriltagVisionIOPhoton(CameraId.INTAKE_LEFT.toString());
        intakeRightIo = new ApriltagVisionIOPhoton(CameraId.INTAKE_RIGHT.toString());
        reefBackLeftIo = new ApriltagVisionIOPhoton(CameraId.REEF_BACK_LEFT.toString());
        reefBackRightIo = new ApriltagVisionIOPhoton(CameraId.REEF_BACK_RIGHT.toString());
      }

      case SIM -> {
        visionSystemSim = new VisionSystemSim("apriltag");
        visionSystemSim.addAprilTags(Field.APRILTAG_LAYOUT.getLayout());

        intakeLeftIo =
            new ApriltagVisionIOPhotonSim(
                CameraId.INTAKE_LEFT.toString(),
                KS1A293_1280_800(),
                ApriltagVisionConfig.INTAKE_LEFT_IN_ROBOT.toTransform3d(),
                visionSystemSim);
        intakeRightIo =
            new ApriltagVisionIOPhotonSim(
                CameraId.INTAKE_RIGHT.toString(),
                KS1A293_1280_800(),
                ApriltagVisionConfig.INTAKE_RIGHT_IN_ROBOT.toTransform3d(),
                visionSystemSim);
        reefBackLeftIo =
            new ApriltagVisionIOPhotonSim(
                CameraId.REEF_BACK_LEFT.toString(),
                KS1A293_1280_800(),
                ApriltagVisionConfig.REEF_BACK_LEFT_IN_ROBOT.toTransform3d(),
                visionSystemSim);
        reefBackRightIo =
            new ApriltagVisionIOPhotonSim(
                CameraId.REEF_BACK_RIGHT.toString(),
                KS1A293_1280_800(),
                ApriltagVisionConfig.REEF_BACK_RIGHT_IN_ROBOT.toTransform3d(),
                visionSystemSim);
      }

      default -> {
        intakeLeftIo = new ApriltagVisionIO() {};
        intakeRightIo = new ApriltagVisionIO() {};
        reefBackLeftIo = new ApriltagVisionIO() {};
        reefBackRightIo = new ApriltagVisionIO() {};
      }
    }

    var warmUpInput = new ApriltagVisionIOInputsAutoLogged();
    warmUpInput.connected = false;
    warmUpInput.ids = new int[] {-1};
    warmUpInput.poseObservations =
        new ApriltagVisionIO.PoseObservation[] {
          new ApriltagVisionIO.PoseObservation(-1, new Pose3d(), 0, 1, 0.0)
        };
    warmUpInput.txTyObservations =
        new ApriltagVisionIO.TxTyObservation[] {
          new ApriltagVisionIO.TxTyObservation(-1, -1, 0.0, 0.0, 0.0)
        };
    Logger.processInputs("ApriltagVision/WarmUp", warmUpInput);
  }

  @Override
  public void periodic() {
    if (Config.MODE == Config.Mode.SIM) {
      // pose in real world or simulated environment with physics engine
      // for most of the time can use pose of wheeled odometry
      var startTime = Timer.getFPGATimestamp();
      visionSystemSim.update(Odometry.getInstance().getWheeledPose());
      Logger.recordOutput(
          "Performance/ApriltagVision/visionSystemSim/update",
          Timer.getFPGATimestamp() - startTime);
    }

    updateInputs();

    intakeLeftOfflineAlert.set(!intakeLeftInputs.connected);
    intakeRightOfflineAlert.set(!intakeRightInputs.connected);
    reefBackLeftOfflineAlert.set(!reefBackLeftInputs.connected);
    reefBackRightOfflineAlert.set(!reefBackRightInputs.connected);

    allGoodVisionObservations.clear();
    allSingleTagVisionObservations.clear();
    allGoodRobotInField.clear();
    allGoodRobotInField2d.clear();
    allBadRobotInField.clear();
    allUsedTagInField.clear();

    allSingleTagVisionObservations.put(CameraId.INTAKE_LEFT, new HashMap<>());
    allSingleTagVisionObservations.put(CameraId.INTAKE_RIGHT, new HashMap<>());
    allSingleTagVisionObservations.put(CameraId.REEF_BACK_LEFT, new HashMap<>());
    allSingleTagVisionObservations.put(CameraId.REEF_BACK_RIGHT, new HashMap<>());

    updateRobotInField(
        CameraId.INTAKE_LEFT, intakeLeftInputs, ApriltagVisionConfig.INTAKE_LEFT_IN_ROBOT, true);
    updateRobotInField(
        CameraId.INTAKE_RIGHT, intakeRightInputs, ApriltagVisionConfig.INTAKE_RIGHT_IN_ROBOT, true);
    updateRobotInField(
        CameraId.REEF_BACK_LEFT,
        reefBackLeftInputs,
        ApriltagVisionConfig.REEF_BACK_LEFT_IN_ROBOT,
        true);
    updateRobotInField(
        CameraId.REEF_BACK_RIGHT,
        reefBackRightInputs,
        ApriltagVisionConfig.REEF_BACK_RIGHT_IN_ROBOT,
        true);

    if (anyCameraHasUpdate()) {
      Logger.recordOutput(
          "ApriltagVision/AllBadRobotInField", allBadRobotInField.toArray(Pose3d[]::new));
      Logger.recordOutput(
          "ApriltagVision/AllGoodRobotInField", allGoodRobotInField.toArray(Pose3d[]::new));
      Logger.recordOutput(
          "ApriltagVision/AllGoodRobotInField2d", allGoodRobotInField2d.toArray(Pose2d[]::new));
      Logger.recordOutput(
          "ApriltagVision/AllUsedTagInField", allUsedTagInField.toArray(Pose3d[]::new));

      for (var singleTgaPoses : allSingleTagVisionObservations.entrySet()) {
        var cameraId = singleTgaPoses.getKey();
        singleTgaPoses
            .getValue()
            .values()
            .forEach(pose -> Odometry.getInstance().addSingleTagObservation(cameraId, pose));
      }

      allGoodVisionObservations.stream()
          .sorted(Comparator.comparingDouble(Odometry.VisionObservation::timestamp))
          .forEach(Odometry.getInstance()::addVisionObservation);
    }
  }

  private void updateInputs() {
    intakeLeftIo.updateInputs(intakeLeftInputs);
    intakeRightIo.updateInputs(intakeRightInputs);
    reefBackLeftIo.updateInputs(reefBackLeftInputs);
    reefBackRightIo.updateInputs(reefBackRightInputs);

    Logger.processInputs("ApriltagVision/" + CameraId.INTAKE_LEFT.toString(), intakeLeftInputs);
    Logger.processInputs("ApriltagVision/" + CameraId.INTAKE_RIGHT.toString(), intakeRightInputs);
    Logger.processInputs(
        "ApriltagVision/" + CameraId.REEF_BACK_LEFT.toString(), reefBackLeftInputs);
    Logger.processInputs(
        "ApriltagVision/" + CameraId.REEF_BACK_RIGHT.toString(), reefBackRightInputs);
  }

  private void updateRobotInField(
      CameraId cameraId,
      ApriltagVisionIOInputsAutoLogged inputs,
      Pose3d cameraInRobot,
      boolean isReefCamera) {
    if (!(inputs.connected
        && inputs.hasUpdate
        && inputs.hasTargets
        && inputs.poseObservations.length != 0
        && inputs.ids.length != 0
        && inputs.txTyObservations.length != 0)) {
      return;
    }

    for (final var poseObservation : inputs.poseObservations) {
      if (poseObservation.ambiguity() > maxAllowedAmbiguity.get()) {
        continue;
      }

      final var robotInField =
          poseObservation.cameraInField().transformBy(cameraInRobot.toTransform3d().inverse());

      if (robotInField.getX() < -ApriltagVisionConfig.FIELD_BORDER_THRESHOLD_METER
          || robotInField.getX() > Field.LENGTH + ApriltagVisionConfig.FIELD_BORDER_THRESHOLD_METER
          || robotInField.getY() < -ApriltagVisionConfig.FIELD_BORDER_THRESHOLD_METER
          || robotInField.getY() > Field.WIDTH + ApriltagVisionConfig.FIELD_BORDER_THRESHOLD_METER
          || robotInField.getZ() > ApriltagVisionConfig.ROBOT_POSE_Z_THRESHOLD_METER
          || robotInField.getZ() < -ApriltagVisionConfig.ROBOT_POSE_Z_THRESHOLD_METER) {
        allBadRobotInField.add(robotInField);
        continue;
      }

      final var xyStdDev =
          xyStdMeter.get()
              * Math.pow(poseObservation.avgDistance(), 2)
              / poseObservation.tagCount();

      final var thetaStdDev =
          poseObservation.tagCount() > 1
              ? Units.degreesToRadians(thetaStdDegree.get())
                  * Math.pow(poseObservation.avgDistance(), 2)
                  / poseObservation.tagCount()
              : Double.POSITIVE_INFINITY;

      final var robotInField2d = robotInField.toPose2d();
      allGoodVisionObservations.add(
          new Odometry.VisionObservation(
              poseObservation.timestamp(),
              robotInField2d,
              VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev)));
      allGoodRobotInField.add(robotInField);
      allGoodRobotInField2d.add(robotInField2d);
    }

    for (final var id : inputs.ids) {
      allUsedTagInField.add(Field.APRILTAG_LAYOUT.getLayout().getTagPose(id).get());
    }

    if (!isReefCamera) {
      return;
    }

    final var odometry = Odometry.getInstance();
    final var wheeledPose = odometry.getWheeledPose();
    final var estimatedPose = odometry.getEstimatedPose();

    for (final var observation : inputs.txTyObservations) {
      final var oldWheeledPose = odometry.getWheeledPoseByTimestamp(observation.timestamp());

      if (oldWheeledPose.isEmpty()) {
        continue;
      }
      final var robotRotation =
          estimatedPose
              .transformBy(new Transform2d(wheeledPose, oldWheeledPose.get()))
              .getRotation();

      final var camToTagTranslation =
          new Pose3d(
                  Translation3d.kZero, new Rotation3d(0, observation.tyRad(), -observation.txRad()))
              .transformBy(
                  new Transform3d(
                      new Translation3d(observation.distance(), 0, 0), Rotation3d.kZero))
              .getTranslation()
              .rotateBy(new Rotation3d(0, cameraInRobot.getRotation().getY(), 0))
              .toTranslation2d();

      final var camToTagRotation =
          robotRotation.plus(
              cameraInRobot.toPose2d().getRotation().plus(camToTagTranslation.getAngle()));

      final var tagInField = Field.APRILTAG_LAYOUT.getLayout().getTagPose(observation.id()).get();
      final var cameraInFieldTranslation =
          new Pose2d(tagInField.toPose2d().getTranslation(), camToTagRotation.plus(Rotation2d.kPi))
              .transformBy(GeomUtil.toTransform2d(camToTagTranslation.getNorm(), 0.0))
              .getTranslation();
      var robotInField =
          new Pose2d(
                  cameraInFieldTranslation,
                  robotRotation.plus(cameraInRobot.toPose2d().getRotation()))
              .transformBy(new Transform2d(cameraInRobot.toPose2d(), Pose2d.kZero));
      robotInField = new Pose2d(robotInField.getTranslation(), robotRotation);

      var singleTagVisionObservations = allSingleTagVisionObservations.get(cameraId);
      if (!singleTagVisionObservations.containsKey(observation.id())
          || observation.timestamp()
              > singleTagVisionObservations.get(observation.id()).timestamp()) {
        singleTagVisionObservations.put(
            observation.id(),
            new Odometry.SingleTagVisionObservation(
                observation.timestamp(), robotInField, observation.id(), observation.distance()));
      }
    }
  }

  private boolean singleCameraHasTagById(ApriltagVisionIOInputsAutoLogged inputs, int id) {
    if (!inputs.connected || inputs.ids == null) {
      return false;
    }
    return Arrays.stream(inputs.ids).anyMatch(seemedId -> seemedId == id);
  }

  public boolean reefCameraHasTagById(int id) {
    return singleCameraHasTagById(reefBackLeftInputs, id)
        || singleCameraHasTagById(reefBackRightInputs, id)
        || singleCameraHasTagById(intakeLeftInputs, id)
        || singleCameraHasTagById(intakeRightInputs, id);
  }

  private boolean anyCameraHasUpdate() {
    return reefBackLeftInputs.hasUpdate
        || reefBackRightInputs.hasUpdate
        || intakeLeftInputs.hasUpdate
        || intakeRightInputs.hasUpdate;
  }
}
