package org.wpilib.vision.apriltag;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import java.io.IOException;
import java.nio.file.Path;
import java.util.List;
import org.wpilib.fields.Field;
import org.wpilib.fields.FieldTag;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Quaternion;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation3d;

/**
 * Bridges PhotonVision alpha-2's removed layout API to WPILib 2027's {@link Field}.
 *
 * <p>Remove this class and its companion compatibility types after upgrading PhotonVision. See
 * doc/PHOTONVISION-COMPAT.md.
 */
public final class AprilTagFieldLayout {
  private static final ObjectMapper MAPPER = new ObjectMapper();
  private final Field field;

  public AprilTagFieldLayout(Path path) throws IOException {
    JsonNode root = MAPPER.readTree(path.toFile());
    JsonNode dimensions = root.required("field");
    List<FieldTag> tags = new java.util.ArrayList<>();
    for (JsonNode tag : root.required("tags")) {
      JsonNode translation = tag.required("pose").required("translation");
      JsonNode quaternion = tag.required("pose").required("rotation").required("quaternion");
      Pose3d pose =
          new Pose3d(
              new Translation3d(
                  translation.required("x").asDouble(),
                  translation.required("y").asDouble(),
                  translation.required("z").asDouble()),
              new Rotation3d(
                  new Quaternion(
                      quaternion.required("W").asDouble(),
                      quaternion.required("X").asDouble(),
                      quaternion.required("Y").asDouble(),
                      quaternion.required("Z").asDouble())));
      tags.add(new FieldTag(tag.required("ID").asInt(), pose));
    }
    String name = path.getFileName().toString().replaceFirst("\\.json$", "");
    field =
        new Field(
            name,
            name.matches("^\\d{4}.*") ? name.substring(0, 4) : "custom",
            name,
            null,
            dimensions.required("length").asDouble(),
            dimensions.required("width").asDouble(),
            "frc",
            tags);
  }

  private AprilTagFieldLayout(Field field) {
    this.field = field;
  }

  public static AprilTagFieldLayout loadField(AprilTagFields field) {
    return new AprilTagFieldLayout(Field.loadField(field.field));
  }

  public List<AprilTag> getTags() {
    return field.getTags().stream().map(tag -> new AprilTag(tag.getID(), tag.getPose())).toList();
  }

  public java.util.Optional<Pose3d> getTagPose(int id) {
    return field.getTagPose(id);
  }

  public Pose3d getOrigin() {
    return field.getOrigin();
  }

  public double getFieldLength() {
    return field.getFieldLength();
  }

  public double getFieldWidth() {
    return field.getFieldWidth();
  }
}
