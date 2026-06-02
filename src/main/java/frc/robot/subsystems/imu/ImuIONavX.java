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

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.subsystems.drive.PhoenixOdometryThread;
import frc.robot.subsystems.drive.SwerveConstants;
import java.util.Iterator;
import java.util.Queue;

/** IMU IO for NavX. Primitive-only: yaw/rate in radians, accel in m/s^2, jerk in m/s^3. */
public class ImuIONavX implements ImuIO {

  // Define the NavX Hardware
  private final AHRS navx;

  // Queues
  private final Queue<Double> yawPositionDegQueue;
  private final Queue<Double> yawTimestampQueue;

  // Previous accel for jerk calculation (m/s/s)
  private Translation3d prevAcc = Translation3d.kZero;
  private long prevTimestampNs = 0L;

  // Reusable buffers for queue-drain (to avoid using streams)
  private double[] odomTsBuf = new double[8];
  private double[] odomYawRadBuf = new double[8];

  /** Constructor */
  public ImuIONavX() {
    // Initialize NavX over SPI
    navx = new AHRS(NavXComType.kMXP_SPI, (byte) SwerveConstants.kOdometryFrequency);

    // Alliance-based adjustment (your original behavior)
    if (DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Red) {
      navx.setAngleAdjustment(180.0);
    } else {
      navx.setAngleAdjustment(0.0);
    }
    navx.reset();

    yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
    yawPositionDegQueue = PhoenixOdometryThread.getInstance().registerSignal(navx::getYaw);
  }

  /** Update the Inputs */
  @Override
  public void updateInputs(ImuIOInputs inputs) {
    final long start = System.nanoTime();

    // Load the nanosecond timestamp
    inputs.timestampNs = start;

    inputs.connected = navx.isConnected();

    // Your original sign convention:
    // yawPosition = Rotation2d.fromDegrees(-navx.getAngle());
    // yawRate = -navx.getRawGyroZ()
    //
    // NavX:
    //  - getAngle() is degrees (continuous)
    //  - getRawGyroZ() is deg/sec
    inputs.yawPositionRad = Units.degreesToRadians(-navx.getAngle());
    inputs.yawRateRadPerSec = Units.degreesToRadians(-navx.getRawGyroZ());

    // World linear accel (NavX returns "g" typically); convert to m/s/s
    inputs.linearAccel =
        new Translation3d(
            navx.getWorldLinearAccelX() * Constants.G_TO_MPS2,
            navx.getWorldLinearAccelY() * Constants.G_TO_MPS2,
            navx.getWorldLinearAccelZ() * Constants.G_TO_MPS2);

    // Jerk computed as (delta accel) / dt
    if (prevTimestampNs != 0L) {
      final double dt = (start - prevTimestampNs) * 1e-9;
      if (dt > 1e-6) {
        inputs.linearJerk = inputs.linearAccel.minus(prevAcc).div(dt);
      }
    }

    // Load "previous values" for the next loop
    prevTimestampNs = start;
    prevAcc = inputs.linearAccel;

    // Drain odometry queues to primitive arrays (timestamps == doubles; yaws == degrees)
    final int n = drainOdomQueues();
    if (n > 0) {
      // If there's anything to drain...
      final double[] tsOut = new double[n];
      final double[] yawOut = new double[n];
      System.arraycopy(odomTsBuf, 0, tsOut, 0, n);
      System.arraycopy(odomYawRadBuf, 0, yawOut, 0, n);
      inputs.odometryYawTimestamps = tsOut;
      inputs.odometryYawPositionsRad = yawOut;
    } else {
      // ...otherwise return empty arrays
      inputs.odometryYawTimestamps = new double[] {};
      inputs.odometryYawPositionsRad = new double[] {};
    }

    // Compute how long this took in seconds
    final long end = System.nanoTime();
    inputs.latencySeconds = (end - start) * 1e-9;
  }

  /**
   * Zero the YAW to this radian value
   *
   * @param yawRad The radian value to which to zero
   */
  @Override
  public void zeroYawRad(double yawRad) {
    navx.setAngleAdjustment(Units.radiansToDegrees(yawRad));
    navx.zeroYaw();

    // Reset jerk history so you don't spike on the next frame
    prevTimestampNs = 0L;
    prevAcc = Translation3d.kZero;
  }

  /**
   * Drain the Odometry Queues into a Buffer
   *
   * <p>Private function that does the heavy lifting of draining the queues
   */
  private int drainOdomQueues() {
    final int nTs = yawTimestampQueue.size();
    final int nYaw = yawPositionDegQueue.size();
    final int n = Math.min(nTs, nYaw);
    if (n <= 0) {
      yawTimestampQueue.clear();
      yawPositionDegQueue.clear();
      return 0;
    }

    ensureOdomCapacity(n);

    final Iterator<Double> itT = yawTimestampQueue.iterator();
    final Iterator<Double> itY = yawPositionDegQueue.iterator();

    int i = 0;
    while (i < n && itT.hasNext() && itY.hasNext()) {
      odomTsBuf[i] = itT.next();

      // queue provides degrees (navx::getYaw). Apply your sign convention (-d) then rad.
      final double yawDeg = -itY.next();
      odomYawRadBuf[i] = Units.degreesToRadians(yawDeg);

      i++;
    }

    yawTimestampQueue.clear();
    yawPositionDegQueue.clear();
    return i;
  }

  /**
   * Check that buffer is big enough for this queue
   *
   * <p>Private function that ensures odometry buffer capacity
   */
  private void ensureOdomCapacity(int n) {
    if (odomTsBuf.length >= n) return;
    int newCap = odomTsBuf.length;
    while (newCap < n) newCap *= 2;
    odomTsBuf = new double[newCap];
    odomYawRadBuf = new double[newCap];
  }
}
