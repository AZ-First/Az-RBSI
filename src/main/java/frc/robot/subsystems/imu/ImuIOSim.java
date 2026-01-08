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

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.LinkedList;
import java.util.Queue;

/** Simulated IMU for full robot simulation & replay logging */
public class ImuIOSim implements ImuIO {

  private double yawDeg = 0.0;
  private double yawRateDps = 0.0;
  private Translation3d linearAccel = Translation3d.kZero;

  private final Queue<Double> odomTimestamps = new LinkedList<>();
  private final Queue<Double> odomYaws = new LinkedList<>();

  private final double loopPeriodSecs;

  public ImuIOSim(double loopPeriodSecs) {
    this.loopPeriodSecs = loopPeriodSecs;
  }

  @Override
  public void updateInputs(ImuIOInputs inputs) {
    // Populate raw IMU readings
    inputs.connected = true;
    inputs.yawPosition = Rotation2d.fromDegrees(yawDeg);
    inputs.yawVelocityRadPerSec = RadiansPerSecond.of(yawRateDps);
    inputs.linearAccel = linearAccel;

    // Maintain odometry history for latency/logging
    double now = System.currentTimeMillis() / 1000.0;
    odomTimestamps.add(now);
    odomYaws.add(yawDeg);

    while (odomTimestamps.size() > 50) odomTimestamps.poll();
    while (odomYaws.size() > 50) odomYaws.poll();

    // Fill the provided inputs object
    inputs.odometryYawTimestamps = odomTimestamps.stream().mapToDouble(d -> d).toArray();
    inputs.odometryYawPositions =
        odomYaws.stream().map(d -> Rotation2d.fromDegrees(d)).toArray(Rotation2d[]::new);
  }

  @Override
  public void zeroYaw(Rotation2d yaw) {
    yawDeg = yaw.getDegrees();
  }

  // --- Simulation helpers to update the IMU state ---
  public void setYawDeg(double deg) {
    yawDeg = deg;
  }

  public void setYawRateDps(double dps) {
    yawRateDps = dps;
  }

  public void setLinearAccel(Translation3d accelMps2) {
    linearAccel = accelMps2;
  }

  /** Optional: integrate yaw from yawRate over loop period */
  public void integrateYaw() {
    yawDeg += yawRateDps * loopPeriodSecs;
  }
}
