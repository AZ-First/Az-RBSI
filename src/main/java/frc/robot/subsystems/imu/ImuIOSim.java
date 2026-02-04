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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;

/** Simulated IMU for full robot simulation & replay logging */
public class ImuIOSim implements ImuIO {

  // --- AUTHORITATIVE SIM STATE (PRIMITIVES) ---
  private double yawRad = 0.0;
  private double yawRateRadPerSec = 0.0;
  private double ax = 0.0, ay = 0.0, az = 0.0; // m/s^2

  // --- ODOMETRY HISTORY (PRIMITIVE RING BUFFER) ---
  private static final int ODOM_CAP = 50;
  private final double[] odomTs = new double[ODOM_CAP];
  private final double[] odomYawRad = new double[ODOM_CAP];
  private int odomSize = 0;
  private int odomHead = 0; // next write index

  public ImuIOSim() {}

  // ---------------- SIMULATION INPUTS (PUSH) ----------------
  @Override
  public void simulationSetYawRad(double yawRad) {
    this.yawRad = yawRad;
  }

  @Override
  public void simulationSetOmegaRadPerSec(double omegaRadPerSec) {
    this.yawRateRadPerSec = omegaRadPerSec;
  }

  @Override
  public void simulationSetLinearAccelMps2(double ax, double ay, double az) {
    this.ax = ax;
    this.ay = ay;
    this.az = az;
  }

  // ---------------- IO UPDATE (PULL) ----------------
  @Override
  public void updateInputs(ImuIOInputs inputs) {
    inputs.connected = true;
    inputs.timestampNs = System.nanoTime();

    // Authoritative sim state
    inputs.yawPositionRad = yawRad;
    inputs.yawRateRadPerSec = yawRateRadPerSec;
    inputs.linearAccel = new Translation3d(ax, ay, az);

    // Jerk: SIM doesn't have a prior accel here unless you want it; set to 0 by default.
    // If you do want jerk, you can add prevAx/prevAy/prevAz + dt just like the real IO.
    inputs.linearJerk = Translation3d.kZero;

    // Maintain odometry history
    pushOdomSample(Timer.getFPGATimestamp(), yawRad);

    // Export odometry arrays (copy out in chronological order)
    final int n = odomSize;
    final double[] tsOut = new double[n];
    final double[] yawOut = new double[n];

    // Oldest sample index:
    int idx = (odomHead - odomSize);
    while (idx < 0) idx += ODOM_CAP;

    for (int i = 0; i < n; i++) {
      tsOut[i] = odomTs[idx];
      yawOut[i] = odomYawRad[idx];
      idx++;
      if (idx == ODOM_CAP) idx = 0;
    }

    inputs.odometryYawTimestamps = tsOut;
    inputs.odometryYawPositionsRad = yawOut;

    // Optional: SIM logging (primitive-friendly)
    Logger.recordOutput("IMU/YawRad", yawRad);
    Logger.recordOutput("IMU/YawDeg", Units.radiansToDegrees(yawRad));
    Logger.recordOutput("IMU/YawRateDps", Units.radiansToDegrees(yawRateRadPerSec));
  }

  /**
   * Zero the YAW to this radian value
   *
   * @param yawRad The radian value to which to zero
   */
  @Override
  public void zeroYawRad(double yawRad) {
    this.yawRad = yawRad;
    this.yawRateRadPerSec = 0.0;
  }

  private void pushOdomSample(double timestampSec, double yawRad) {
    odomTs[odomHead] = timestampSec;
    odomYawRad[odomHead] = yawRad;

    odomHead++;
    if (odomHead == ODOM_CAP) odomHead = 0;

    if (odomSize < ODOM_CAP) odomSize++;
  }
}
