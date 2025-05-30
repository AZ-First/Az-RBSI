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

import edu.wpi.first.apriltag.AprilTagFieldLayout;

public class VisionIOPhoton implements VisionIO {
  final SimCamera scoringCamera, frontCamera;

  public VisionIOPhoton(AprilTagFieldLayout apriltagLayout) {
    scoringCamera =
        new SimCamera("scoring", VisionSubsystem.ROBOT_TO_SCORING_CAMERA, apriltagLayout);
    frontCamera = new SimCamera("front", VisionSubsystem.ROBOT_TO_FRONT_CAMERA, apriltagLayout);
  }

  @Override
  public void update(VisionIOInputs inputs) {
    inputs.scoringCameraConnected = true;
    inputs.frontCameraConnected = true;
    inputs.backCameraConnected = true;

    inputs.scoringPoseEstimate = scoringCamera.getPoseEstimate();
    if (inputs.scoringPoseEstimate != null) {
      inputs.scoringTimestamp = inputs.scoringPoseEstimate.timestampSeconds;
      // We're not fully simulating the other camera, but we can use the scoring camera's timestamp
      inputs.backTimestamp = inputs.scoringTimestamp;
    }
    inputs.scoringFiducials = scoringCamera.getRawFiducials();
    inputs.frontPoseEstimate = frontCamera.getPoseEstimate();
  }

  @Override
  public void simulationPeriodic() {
    scoringCamera.update();
    frontCamera.update();
  }
}
