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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.drive.PhoenixOdometryThread;
import frc.robot.subsystems.drive.SwerveConstants;
import java.util.Iterator;
import java.util.Queue;

/** IMU IO for NavX. Primitive-only: yaw/rate in radians, accel in m/s^2, jerk in m/s^3. */
public class ImuIONavX implements ImuIO {
  private static final double DEG_TO_RAD = Math.PI / 180.0;
  private static final double G_TO_MPS2 = 9.80665;

  private final AHRS navx;

  // Phoenix odometry queues (boxed Doubles, but we drain without streams)
  private final Queue<Double> yawPositionDegQueue;
  private final Queue<Double> yawTimestampQueue;

  // Previous accel (m/s^2) for jerk
  private double prevAx = 0.0, prevAy = 0.0, prevAz = 0.0;
  private long prevTimestampNs = 0L;

  // Reusable buffers for queue drain
  private double[] odomTsBuf = new double[8];
  private double[] odomYawRadBuf = new double[8];

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

  @Override
  public void updateInputs(ImuIOInputs inputs) {
    final long start = System.nanoTime();

    inputs.connected = navx.isConnected();

    // Your original sign convention:
    // yawPosition = Rotation2d.fromDegrees(-navx.getAngle());
    // yawRate = -navx.getRawGyroZ()
    //
    // NavX:
    //  - getAngle() is degrees (continuous)
    //  - getRawGyroZ() is deg/sec
    final double yawDeg = -navx.getAngle();
    final double yawRateDegPerSec = -navx.getRawGyroZ();

    inputs.yawPositionRad = yawDeg * DEG_TO_RAD;
    inputs.yawRateRadPerSec = yawRateDegPerSec * DEG_TO_RAD;

    // World linear accel (NavX returns "g" typically). Convert to m/s^2.
    final double ax = navx.getWorldLinearAccelX() * G_TO_MPS2;
    final double ay = navx.getWorldLinearAccelY() * G_TO_MPS2;
    final double az = navx.getWorldLinearAccelZ() * G_TO_MPS2;

    inputs.linearAccelX = ax;
    inputs.linearAccelY = ay;
    inputs.linearAccelZ = az;

    // Jerk
    if (prevTimestampNs != 0L) {
      final double dt = (start - prevTimestampNs) * 1e-9;
      if (dt > 1e-6) {
        final double invDt = 1.0 / dt;
        inputs.jerkX = (ax - prevAx) * invDt;
        inputs.jerkY = (ay - prevAy) * invDt;
        inputs.jerkZ = (az - prevAz) * invDt;
      }
    }

    prevTimestampNs = start;
    prevAx = ax;
    prevAy = ay;
    prevAz = az;

    inputs.timestampNs = start;

    // Odometry history
    final int n = drainOdomQueues();
    if (n > 0) {
      final double[] tsOut = new double[n];
      final double[] yawOut = new double[n];
      System.arraycopy(odomTsBuf, 0, tsOut, 0, n);
      System.arraycopy(odomYawRadBuf, 0, yawOut, 0, n);
      inputs.odometryYawTimestamps = tsOut;
      inputs.odometryYawPositionsRad = yawOut;
    } else {
      inputs.odometryYawTimestamps = new double[] {};
      inputs.odometryYawPositionsRad = new double[] {};
    }

    final long end = System.nanoTime();
    inputs.latencySeconds = (end - start) * 1e-9;
  }

  @Override
  public void zeroYawRad(double yawRad) {
    navx.setAngleAdjustment(yawRad / DEG_TO_RAD);
    navx.zeroYaw();

    // Reset jerk history so you don't spike on the next frame
    prevTimestampNs = 0L;
    prevAx = prevAy = prevAz = 0.0;
  }

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
      odomYawRadBuf[i] = yawDeg * DEG_TO_RAD;

      i++;
    }

    yawTimestampQueue.clear();
    yawPositionDegQueue.clear();
    return i;
  }

  private void ensureOdomCapacity(int n) {
    if (odomTsBuf.length >= n) return;
    int newCap = odomTsBuf.length;
    while (newCap < n) newCap *= 2;
    odomTsBuf = new double[newCap];
    odomYawRadBuf = new double[newCap];
  }

  // /**
  //  * Zero the NavX
  //  *
  //  * <p>This method should always rezero the pigeon in ALWAYS-BLUE-ORIGIN orientation. Testing,
  //  * however, shows that it's not doing what I think it should be doing. There is likely
  //  * interference with something else in the odometry
  //  */
  // @Override
  // public void zero() {
  //   // With the Pigeon facing forward, forward depends on the alliance selected.
  //   // Set Angle Adjustment based on alliance
  //   if (DriverStation.getAlliance().get() == Alliance.Blue) {
  //     navx.setAngleAdjustment(0.0);
  //   } else {
  //     navx.setAngleAdjustment(180.0);
  //   }
  //   System.out.println("Setting YAW to " + navx.getAngleAdjustment());
  //   navx.zeroYaw();
  // }

}
