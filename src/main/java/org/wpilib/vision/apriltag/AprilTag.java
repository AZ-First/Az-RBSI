package org.wpilib.vision.apriltag;

import org.wpilib.math.geometry.Pose3d;

/** Temporary PhotonVision alpha-2 binary compatibility type. See doc/PHOTONVISION-COMPAT.md. */
public final class AprilTag {
  public final int ID;
  public final Pose3d pose;

  public AprilTag(int id, Pose3d pose) {
    this.ID = id;
    this.pose = pose;
  }
}
