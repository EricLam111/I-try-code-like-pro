package com.nextinnovation.team8214.subsystem.vision;

import com.nextinnovation.team8214.Field;
import edu.wpi.first.math.geometry.Transform3d;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class ApriltagVisionIOPhotonSim extends ApriltagVisionIOPhoton {
  ApriltagVisionIOPhotonSim(
      String cameraName,
      SimCameraProperties simCameraProperties,
      Transform3d robot2Camera,
      VisionSystemSim visionSystemSim) {
    super(cameraName);

    var sim =
        new PhotonCameraSim(super.camera, simCameraProperties, Field.APRILTAG_LAYOUT.getLayout());
    sim.enableProcessedStream(true);

    visionSystemSim.addCamera(sim, robot2Camera);
  }
}
