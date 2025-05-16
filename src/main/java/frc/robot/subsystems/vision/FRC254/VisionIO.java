// Copyright (c) 2024 FRC 254
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

import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {

  @AutoLog
  class VisionIOInputs {
    public boolean turretCameraSeesTarget;

    public boolean elevatorCameraSeesTarget;

    public FiducialObservation[] turretCameraFiducialObservations;

    public FiducialObservation[] elevatorCameraFiducialObservations;

    public MegatagPoseEstimate turretCameraMegatagPoseEstimate;

    public int turretCameraMegatagCount;

    public MegatagPoseEstimate elevatorCameraMegatagPoseEstimate;

    public int elevatorCameraMegatagCount;

    public MegatagPoseEstimate turretCameraMegatag2PoseEstimate;

    public MegatagPoseEstimate elevatorCameraMegatag2PoseEstimate;
  }

  void readInputs(VisionIOInputs inputs);

  void pollNetworkTables();
}
