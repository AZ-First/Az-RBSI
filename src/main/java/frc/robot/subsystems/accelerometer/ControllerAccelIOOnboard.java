// Copyright (c) 2024-2027 Az-FIRST
// http://github.com/AZ-First
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

package frc.robot.subsystems.accelerometer;

import org.wpilib.hardware.imu.OnboardIMU;
import org.wpilib.hardware.imu.OnboardIMU.MountOrientation;
import org.wpilib.system.RobotController;

/** Reads the Systemcore's onboard IMU accelerometer. */
public class ControllerAccelIOOnboard implements ControllerAccelIO {
  private final OnboardIMU imu;

  public ControllerAccelIOOnboard() {
    this(new OnboardIMU(MountOrientation.FLAT));
  }

  ControllerAccelIOOnboard(OnboardIMU imu) {
    this.imu = imu;
  }

  @Override
  public void updateInputs(ControllerAccelIOInputs inputs) {
    double x = imu.getAccelX();
    double y = imu.getAccelY();
    double z = imu.getAccelZ();
    inputs.timestampNs = RobotController.getTime();
    inputs.connected = Double.isFinite(x) && Double.isFinite(y) && Double.isFinite(z);
    inputs.xMetersPerSecondSq = inputs.connected ? x : 0.0;
    inputs.yMetersPerSecondSq = inputs.connected ? y : 0.0;
    inputs.zMetersPerSecondSq = inputs.connected ? z : 0.0;
  }
}
