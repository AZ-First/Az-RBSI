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

public class Imu {
  private final ImuIO io;
  private final ImuIO.ImuIOInputs inputs = new ImuIO.ImuIOInputs();

  // Optional per-cycle cached objects (avoid repeated allocations)
  private long cacheStampNs = -1L;
  private Rotation2d cachedYaw = Rotation2d.kZero;
  private Translation3d cachedAccel = Translation3d.kZero;
  private Translation3d cachedJerk = Translation3d.kZero;

  public Imu(ImuIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
  }

  /** Hot-path access: primitive-only snapshot */
  public ImuIO.ImuIOInputs getInputs() {
    return inputs;
  }

  /** Readable boundary: Rotation2d (alloc/cached per timestamp) */
  public Rotation2d getYaw() {
    refreshCachesIfNeeded();
    return cachedYaw;
  }

  /** Readable boundary: Translation3d accel (alloc/cached per timestamp) */
  public Translation3d getLinearAccel() {
    refreshCachesIfNeeded();
    return cachedAccel;
  }

  public Translation3d getJerk() {
    refreshCachesIfNeeded();
    return cachedJerk;
  }

  public void zeroYaw(Rotation2d yaw) {
    io.zeroYawRad(yaw.getRadians());
  }

  private void refreshCachesIfNeeded() {
    final long stamp = inputs.timestampNs;
    if (stamp == cacheStampNs) return;
    cacheStampNs = stamp;

    cachedYaw = Rotation2d.fromRadians(inputs.yawPositionRad);
    cachedAccel = new Translation3d(inputs.linearAccelX, inputs.linearAccelY, inputs.linearAccelZ);
    cachedJerk = new Translation3d(inputs.jerkX, inputs.jerkY, inputs.jerkZ);
  }

  // ---------------- SIM PUSH (primitive-only boundary) ----------------

  /** Simulation: push authoritative yaw (radians) into the IO layer */
  public void simulationSetYawRad(double yawRad) {
    io.simulationSetYawRad(yawRad);
  }

  /** Simulation: push authoritative yaw rate (rad/s) into the IO layer */
  public void simulationSetOmegaRadPerSec(double omegaRadPerSec) {
    io.simulationSetOmegaRadPerSec(omegaRadPerSec);
  }

  /** Simulation: push authoritative linear accel (m/s^2) into the IO layer */
  public void simulationSetLinearAccelMps2(double ax, double ay, double az) {
    io.simulationSetLinearAccelMps2(ax, ay, az);
  }
}
