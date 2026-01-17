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

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import java.util.LinkedList;
import java.util.Queue;
import org.littletonrobotics.junction.Logger;

/** Simulated IMU for full robot simulation & replay logging */
public class ImuIOSim implements ImuIO {

  private Rotation2d yaw = Rotation2d.kZero;
  private double yawRateRadPerSec = 0.0;
  private Translation3d linearAccel = Translation3d.kZero;

  private final Queue<Double> odomTimestamps = new LinkedList<>();
  private final Queue<Double> odomYaws = new LinkedList<>();

  private final double loopPeriodSecs;

  public ImuIOSim(double loopPeriodSecs) {
    this.loopPeriodSecs = loopPeriodSecs;
  }

  @Override
  public void simulationPeriodic(ChassisSpeeds speeds) {
    yawRateRadPerSec = speeds.omegaRadiansPerSecond;
    yaw = yaw.plus(new Rotation2d(yawRateRadPerSec * loopPeriodSecs));

    Logger.recordOutput("IMU/Yaw", yaw);
    Logger.recordOutput("IMU/YawRateDps", Units.radiansToDegrees(yawRateRadPerSec));
  }

  @Override
  public void updateInputs(ImuIOInputs inputs) {
    // Populate raw IMU readings
    inputs.connected = true;
    inputs.yawPosition = yaw;
    inputs.yawVelocityRadPerSec = RadiansPerSecond.of(yawRateRadPerSec);
    inputs.linearAccel = linearAccel;

    // Maintain odometry history for latency/logging
    double now = Timer.getFPGATimestamp();
    odomTimestamps.add(now);
    odomYaws.add(yaw.getDegrees());

    while (odomTimestamps.size() > 50) odomTimestamps.poll();
    while (odomYaws.size() > 50) odomYaws.poll();

    // Fill the provided inputs object
    inputs.odometryYawTimestamps = odomTimestamps.stream().mapToDouble(d -> d).toArray();
    inputs.odometryYawPositions =
        odomYaws.stream().map(d -> Rotation2d.fromDegrees(d)).toArray(Rotation2d[]::new);
  }

  @Override
  public void zeroYaw(Rotation2d yaw) {
    this.yaw = yaw;
  }

  // --- Simulation helpers to update the IMU state ---
  public void setYawDeg(double deg) {
    yaw = Rotation2d.fromDegrees(deg);
  }

  public void setYawRateDps(double dps) {
    yawRateRadPerSec = Units.degreesToRadians(dps);
  }

  public void setLinearAccel(Translation3d accelMps2) {
    linearAccel = accelMps2;
  }
}
