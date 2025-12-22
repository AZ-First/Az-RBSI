// Copyright (c) 2024-2025 FRC 254
// https://github.com/team254
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.vision.FRC254;

import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import frc.robot.util.LimelightHelpers;
import java.nio.ByteBuffer;
import java.util.Arrays;
import java.util.Objects;

/**
 * Represents an observation of a fiducial marker (AprilTag) with position and quality data.
 *
 * @param id The fiducial marker ID
 * @param txnc Normalized horizontal offset (-1 to 1)
 * @param tync Normalized vertical offset (-1 to 1)
 * @param ambiguity Pose ambiguity score (0 = confident, 1 = ambiguous)
 * @param area Target area as percentage of image
 */
public record FiducialObservation(int id, double txnc, double tync, double ambiguity, double area)
    implements StructSerializable {

  /** Converts a Limelight raw fiducial to a FiducialObservation. */
  public static FiducialObservation fromLimelight(LimelightHelpers.RawFiducial fiducial) {
    if (fiducial == null) {
      return null;
    }
    return new FiducialObservation(
        fiducial.id, fiducial.txnc, fiducial.tync, fiducial.ambiguity, fiducial.ta);
  }

  /** Converts an array of Limelight raw fiducials to FiducialObservation array. */
  public static FiducialObservation[] fromLimelight(LimelightHelpers.RawFiducial[] fiducials) {
    if (fiducials == null) {
      return new FiducialObservation[0];
    }
    return Arrays.stream(fiducials)
        .map(FiducialObservation::fromLimelight)
        .filter(Objects::nonNull)
        .toArray(FiducialObservation[]::new);
  }

  public static final Struct<FiducialObservation> struct =
      new Struct<FiducialObservation>() {
        @Override
        public Class<FiducialObservation> getTypeClass() {
          return FiducialObservation.class;
        }

        @Override
        public String getTypeString() {
          return "record:FiducialObservation";
        }

        @Override
        public int getSize() {
          return Integer.BYTES + 4 * Double.BYTES;
        }

        @Override
        public String getSchema() {
          return "int id;double txnc;double tync;double ambiguity";
        }

        @Override
        public FiducialObservation unpack(ByteBuffer bb) {
          int id = bb.getInt();
          double txnc = bb.getDouble();
          double tync = bb.getDouble();
          double ambiguity = bb.getDouble();
          double area = bb.getDouble();
          return new FiducialObservation(id, txnc, tync, ambiguity, area);
        }

        @Override
        public void pack(ByteBuffer bb, FiducialObservation value) {
          bb.putInt(value.id());
          bb.putDouble(value.txnc());
          bb.putDouble(value.tync());
          bb.putDouble(value.ambiguity());
          bb.putDouble(value.area());
        }

        @Override
        public String getTypeName() {
          return "FiducialObservation";
        }
      };
}
