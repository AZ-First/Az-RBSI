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

public class Imu extends VirtualSubsystem {

  // Declare the io and inputs
  private final ImuIO io;
  private final ImuIO.ImuIOInputs inputs = new ImuIO.ImuIOInputs();

  // Per-cycle cached objects (to avoid repeated allocations)
  private long cacheStampNs = -1L;
  private Rotation2d cachedYaw = Rotation2d.kZero;
  private Translation3d cachedAccel = Translation3d.kZero;
  private Translation3d cachedJerk = Translation3d.kZero;

  /** Constructor */
  public Imu(ImuIO io) {
    this.io = io;
  }

  /**
   * Priority value for this virtual subsystem
   *
   * <p>See `frc.robot.util.VirtualSubsystem` for a description of the suggested values for various
   * virtual subsystems.
   */
  @Override
  protected int getPeriodPriority() {
    return -30;
  }

  /** Periodic function to read inputs */
  public void rbsiPeriodic() {
    io.updateInputs(inputs);
  }

  /**
   * Get the inputs objects
   *
   * <p>Hot-path access: primitive-only snapshot
   *
   * @return The inputs objects
   */
  public ImuIO.ImuIOInputs getInputs() {
    return inputs;
  }

  /**
   * Get the current YAW
   *
   * <p>This function updates the caches if needed.
   *
   * @return The current YAW as a Rotation2d object
   */
  public Rotation2d getYaw() {
    refreshCachesIfNeeded();
    return cachedYaw;
  }

  /**
   * Get the current linear acceleration
   *
   * <p>This function updates the caches as needed.
   *
   * @return The current linear acceleration as a Translation3d object
   */
  public Translation3d getLinearAccel() {
    refreshCachesIfNeeded();
    return cachedAccel;
  }

  /**
   * Get the current jerk
   *
   * <p>This function updates the caches ad needed.
   *
   * @return The current jerk as a Translation3d object
   */
  public Translation3d getJerk() {
    refreshCachesIfNeeded();
    return cachedJerk;
  }

  /**
   * Zero the YAW to this input value
   *
   * @param yaw Input YAW
   */
  public void zeroYaw(Rotation2d yaw) {
    io.zeroYawRad(yaw.getRadians());
  }

  /** Refresh the caches from the inputs, if needed */
  private void refreshCachesIfNeeded() {
    final long stamp = inputs.timestampNs;
    if (stamp == cacheStampNs) return;
    cacheStampNs = stamp;

    cachedYaw = Rotation2d.fromRadians(inputs.yawPositionRad);
    cachedAccel = inputs.linearAccel;
    cachedJerk = inputs.linearJerk;
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
