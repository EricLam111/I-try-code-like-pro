package com.nextinnovation.team8214;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.IOException;
import java.nio.file.Path;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

/** Class to store all field constants. Length/Height -> meter, Angle -> degree */
public final class Field {
  public static final double LENGTH = 17.548;
  public static final double WIDTH = 8.052;

  public static final FieldType FIELD_TYPE = FieldType.ANDYMARK;

  @Getter
  @RequiredArgsConstructor
  public enum FieldType {
    ANDYMARK("andymark"),
    WELDED("welded");

    private final String jsonFolder;
  }

  public static final AprilTagLayoutType APRILTAG_LAYOUT = AprilTagLayoutType.NO_BARGE;
  public static final int APRILTAG_COUNT = 22;

  @Getter
  public enum AprilTagLayoutType {
    OFFICIAL("2025-official"),
    NO_BARGE("2025-no-barge"),
    BLUE_REEF("2025-blue-reef"),
    RED_REEF("2025-red-reef");

    AprilTagLayoutType(String name) {
      try {
        layout =
            new AprilTagFieldLayout(
                Path.of(
                    Filesystem.getDeployDirectory().getPath(),
                    "apriltags",
                    FIELD_TYPE.getJsonFolder(),
                    name + ".json"));
      } catch (IOException e) {
        throw new RuntimeException(e);
      }

      try {
        layoutString = new ObjectMapper().writeValueAsString(layout);
      } catch (JsonProcessingException e) {
        throw new RuntimeException("Failed to serialize AprilTag layout JSON " + this);
      }
    }

    private final AprilTagFieldLayout layout;
    private final String layoutString;
  }
}
