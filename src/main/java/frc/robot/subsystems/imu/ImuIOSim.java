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
import edu.wpi.first.wpilibj.Timer;
import java.util.LinkedList;
import java.util.Queue;
import org.littletonrobotics.junction.Logger;

/** Simulated IMU for full robot simulation & replay logging */
public class ImuIOSim implements ImuIO {

  // --- AUTHORITATIVE SIM STATE ---
  private Rotation2d yaw = Rotation2d.kZero;
  private double yawRateRadPerSec = 0.0;
  private Translation3d linearAccel = Translation3d.kZero;

  // --- ODOMETRY HISTORY FOR LOGGING/LATENCY ---
  private final Queue<Double> odomTimestamps = new LinkedList<>();
  private final Queue<Rotation2d> odomYaws = new LinkedList<>();

  public ImuIOSim() {}

  // ---------------- SIMULATION INPUTS (PUSH) ----------------

  /** Set yaw from authoritative physics */
  public void simulationSetYaw(Rotation2d yaw) {
    this.yaw = yaw;
  }

  /** Set angular velocity from authoritative physics */
  public void simulationSetOmega(double omegaRadPerSec) {
    this.yawRateRadPerSec = omegaRadPerSec;
  }

  /** Set linear acceleration from physics (optional) */
  public void setLinearAccel(Translation3d accelMps2) {
    this.linearAccel = accelMps2;
  }

  // ---------------- IO UPDATE (PULL) ----------------

  /** Populate the IMUIOInputs object with the current SIM state */
  @Override
  public void updateInputs(ImuIOInputs inputs) {
    inputs.connected = true;

    // Authoritative physics
    inputs.yawPosition = yaw;
    inputs.yawVelocityRadPerSec = RadiansPerSecond.of(yawRateRadPerSec);
    inputs.linearAccel = linearAccel;

    // Maintain odometry history for logging / latency purposes
    double now = Timer.getFPGATimestamp();
    odomTimestamps.add(now);
    odomYaws.add(yaw);

    while (odomTimestamps.size() > 50) odomTimestamps.poll();
    while (odomYaws.size() > 50) odomYaws.poll();

    inputs.odometryYawTimestamps = odomTimestamps.stream().mapToDouble(d -> d).toArray();
    inputs.odometryYawPositions = odomYaws.toArray(Rotation2d[]::new);

    // Logging for SIM analysis
    Logger.recordOutput("IMU/Yaw", yaw);
    Logger.recordOutput("IMU/YawRateDps", Math.toDegrees(yawRateRadPerSec));
  }

  @Override
  public void zeroYaw(Rotation2d yaw) {
    this.yaw = yaw;
    this.yawRateRadPerSec = 0.0;
  }
}
