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

import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.util.RBSIIO;
import org.littletonrobotics.junction.AutoLog;

/**
 * Single IMU interface exposing all relevant state: orientation, rates, linear accel, jerk, and
 * odometry samples.
 *
 * <p>Primitive-only core: NO WPILib geometry objects, NO Units objects. Conversions happen at the
 * boundary in the Imu subsystem (wrapper methods).
 */
public interface ImuIO extends RBSIIO {

  @AutoLog
  class ImuIOInputs {
    public boolean connected = false;

    /** FPGA-local timestamp when inputs were captured (ns) */
    public long timestampNs = 0L;

    /** Yaw angle (robot frame) in radians */
    public double yawPositionRad = 0.0;

    /** Yaw angular rate in radians/sec */
    public double yawRateRadPerSec = 0.0;

    /** Linear acceleration in robot frame (m/s^2) */
    public Translation3d linearAccel = Translation3d.kZero;

    /** Linear jerk in robot frame (m/s^3) */
    public Translation3d linearJerk = Translation3d.kZero;

    /** Time spent in the IO update call (seconds) */
    public double latencySeconds = 0.0;

    /** Optional odometry samples (timestamps in seconds) */
    public double[] odometryYawTimestamps = new double[] {};

    /** Optional odometry samples (yaw positions in radians) */
    public double[] odometryYawPositionsRad = new double[] {};
  }

  /** Update the current IMU readings into `inputs` */
  default void updateInputs(ImuIOInputs inputs) {}

  /** Zero the yaw to a known field-relative angle (radians) */
  default void zeroYawRad(double yawRad) {}

  /** Simulation-only hooks */
  default void simulationSetYawRad(double yawRad) {}

  default void simulationSetOmegaRadPerSec(double omegaRadPerSec) {}

  default void simulationSetLinearAccelMps2(double ax, double ay, double az) {}
}
