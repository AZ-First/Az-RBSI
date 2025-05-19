// Copyright (c) 2025 FRC 180
// https://github.com/frc180
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

package frc.robot.subsystems.vision.FRC180;

import edu.wpi.first.epilogue.Logged;
import frc.robot.util.LimelightHelpers.PoseEstimate;
import frc.robot.util.LimelightHelpers.RawDetection;
import frc.robot.util.LimelightHelpers.RawFiducial;

@Logged
public interface VisionIO {

  @Logged
  public class VisionIOInputs {
    final RawFiducial[] emptyFiducials = new RawFiducial[0];
    final RawDetection[] emptyDetections = new RawDetection[0];

    boolean scoringCameraConnected = false;
    PoseEstimate scoringPoseEstimate = null;
    RawFiducial[] scoringFiducials = emptyFiducials;
    double scoringTimestamp = 0.0;
    double scoringCPUTemp = 0;
    double scoringTemp = 0;

    boolean frontCameraConnected = false;
    PoseEstimate frontPoseEstimate = null;
    PoseEstimate frontPoseEstimateMT2 = null;
    RawFiducial[] frontFiducials = emptyFiducials;
    double frontTemp = 0;

    boolean backCameraConnected = false;
    RawDetection[] backDetections = emptyDetections;
    double backTimestamp = 0.0;
  }

  public void update(VisionIOInputs inputs);

  public default void simulationPeriodic() {}
}
