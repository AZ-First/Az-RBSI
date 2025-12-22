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

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/** Represents a robot pose estimate from vision with associated uncertainty and metadata. */
public class VisionFieldPoseEstimate {

  private final Pose2d visionRobotPoseMeters;
  private final double timestampSeconds;
  private final Matrix<N3, N1> visionMeasurementStdDevs;
  private final int numTags;

  /**
   * Creates a new vision field pose estimate.
   *
   * @param visionRobotPoseMeters The estimated robot pose on the field in meters
   * @param timestampSeconds The timestamp when this estimate was captured
   * @param visionMeasurementStdDevs Standard deviations representing measurement uncertainty
   * @param numTags Number of AprilTags used in this pose estimate
   */
  public VisionFieldPoseEstimate(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs,
      int numTags) {
    this.visionRobotPoseMeters = visionRobotPoseMeters;
    this.timestampSeconds = timestampSeconds;
    this.visionMeasurementStdDevs = visionMeasurementStdDevs;
    this.numTags = numTags;
  }

  public Pose2d getVisionRobotPoseMeters() {
    return visionRobotPoseMeters;
  }

  public double getTimestampSeconds() {
    return timestampSeconds;
  }

  public Matrix<N3, N1> getVisionMeasurementStdDevs() {
    return visionMeasurementStdDevs;
  }

  public int getNumTags() {
    return numTags;
  }
}
