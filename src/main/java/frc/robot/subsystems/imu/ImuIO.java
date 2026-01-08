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
import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.AutoLog;

/**
 * Single IMU interface exposing all relevant state: orientation, rates, linear accel, and odometry.
 */
public interface ImuIO {

  @AutoLog
  public static class ImuIOInputs {
    public boolean connected = false;

    // Timestamp
    public long timestampNs = 0;

    // Gyro
    public Rotation2d yawPosition = Rotation2d.kZero;
    public AngularVelocity yawVelocityRadPerSec = RadiansPerSecond.of(0.0);

    // Linear acceleration and jerk in robot frame (m/s^2;  m/s^3)
    public Translation3d linearAccel = Translation3d.kZero;
    public Translation3d jerk = Translation3d.kZero;

    public double latencySeconds = 0.0;

    // Odometry tracking (optional, for external odometry/vision fusion)
    public double[] odometryYawTimestamps = new double[] {};
    public Rotation2d[] odometryYawPositions = new Rotation2d[] {};
  }

  /** Update the current IMU readings into `inputs` */
  default void updateInputs(ImuIOInputs inputs) {}

  /** Zero the yaw to a known field-relative angle */
  default void zeroYaw(Rotation2d yaw) {}
}
