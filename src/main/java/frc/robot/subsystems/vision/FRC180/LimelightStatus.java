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

import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.LimelightHelpers;

public class LimelightStatus implements CameraStatus {

  private final String limelightName;

  private double limelightHeartbeat = 0;
  private double lastHeartbeatTime = 0;
  private boolean limelightConnected = false;

  public LimelightStatus(String limelightName) {
    this.limelightName = limelightName;
  }

  @Override
  public void update() {
    double newHeartbeat = LimelightHelpers.getLimelightNTDouble(limelightName, "hb");
    if (newHeartbeat > limelightHeartbeat) {
      limelightConnected = true;
      limelightHeartbeat = newHeartbeat;
      lastHeartbeatTime = Timer.getFPGATimestamp();
    } else if (Timer.getFPGATimestamp() - lastHeartbeatTime >= 1) {
      limelightConnected = false;
    }
  }

  @Override
  public boolean isConnected() {
    return limelightConnected;
  }
}
