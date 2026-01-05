package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.IOException;
import java.nio.file.Path;
import lombok.Getter;

public class AprilTagLayout {

  /** AprilTag Field Layout ************************************************ */
  /* SEASON SPECIFIC! -- This section is for 2025 (Reefscape) */

  public static final double aprilTagWidth = Inches.of(6.50).in(Meters);

  public static final String aprilTagFamily = "36h11";
  public static final AprilTagLayoutType defaultAprilTagType = AprilTagLayoutType.OFFICIAL;

  public static final AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  @Getter
  public enum AprilTagLayoutType {
    OFFICIAL("2025-official");

    // SPEAKERS_ONLY("2024-speakers"),
    // AMPS_ONLY("2024-amps"),
    // WPI("2024-wpi");

    private AprilTagLayoutType(String name) {
      if (Constants.disableHAL) {
        layout = null;
      } else {
        try {
          layout =
              new AprilTagFieldLayout(
                  Path.of(Filesystem.getDeployDirectory().getPath(), "apriltags", name + ".json"));
        } catch (IOException e) {
          throw new RuntimeException(e);
        }
      }
      if (layout == null) {
        layoutString = "";
      } else {
        try {
          layoutString = new ObjectMapper().writeValueAsString(layout);
        } catch (JsonProcessingException e) {
          throw new RuntimeException(
              "Failed to serialize AprilTag layout JSON " + toString() + "for PhotonVision");
        }
      }
    }

    private final AprilTagFieldLayout layout;
    private final String layoutString;
  }
}
