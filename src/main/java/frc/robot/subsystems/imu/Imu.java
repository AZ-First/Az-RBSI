// Copyright (c) 2024-2026 Az-FIRST
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
//
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.imu;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.util.VirtualSubsystem;
import org.littletonrobotics.junction.Logger;

public class Imu extends VirtualSubsystem {

  private final ImuIO io;
  private final ImuIOInputsAutoLogged inputs = new ImuIOInputsAutoLogged();

  public Imu(ImuIO io) {
    this.io = io;
  }

  @Override
  protected void rbsiPeriodic() {
    io.updateInputs(inputs);
    Logger.processInputs("IMU", inputs); // optional but useful
  }

  public ImuIO.ImuIOInputs getInputs() {
    return inputs;
  }

  // Pass-throughs so Drive can still control the IMU
  public void zeroYaw(Rotation2d yaw) {
    io.zeroYaw(yaw);
  }

  public void simulationSetYaw(Rotation2d yaw) {
    io.simulationSetYaw(yaw);
  }

  public void simulationSetOmega(double omegaRadPerSec) {
    io.simulationSetOmega(omegaRadPerSec);
  }

  public void setLinearAccel(Translation3d accelMps2) {
    io.setLinearAccel(accelMps2);
  }
}
