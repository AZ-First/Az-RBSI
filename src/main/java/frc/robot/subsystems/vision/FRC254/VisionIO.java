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

import edu.wpi.first.math.geometry.Pose3d;

/** Interface for vision system hardware abstraction. */
public interface VisionIO {

  /** Container for all vision input data. */
  class VisionIOInputs {
    /** Input data from a single camera. */
    public static class CameraInputs {
      public boolean seesTarget;
      public FiducialObservation[] fiducialObservations;
      public MegatagPoseEstimate megatagPoseEstimate;
      public MegatagPoseEstimate megatag2PoseEstimate;
      public int megatag2Count;
      public int megatagCount;
      public Pose3d pose3d;
      public double[] standardDeviations =
          new double[12]; // [MT1x, MT1y, MT1z, MT1roll, MT1pitch, MT1Yaw, MT2x,
      // MT2y, MT2z, MT2roll, MT2pitch, MT2yaw]
    }

    public CameraInputs cameraA = new CameraInputs();
    public CameraInputs cameraB = new CameraInputs();
  }

  void readInputs(VisionIOInputs inputs);
}
