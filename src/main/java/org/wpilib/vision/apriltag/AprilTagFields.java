package org.wpilib.vision.apriltag;

import org.wpilib.fields.Fields;

/** Temporary PhotonVision alpha-2 binary compatibility type. See doc/PHOTONVISION-COMPAT.md. */
public enum AprilTagFields {
  kDefaultField(Fields.DEFAULT_FIELD);

  final Fields field;

  AprilTagFields(Fields field) {
    this.field = field;
  }
}
