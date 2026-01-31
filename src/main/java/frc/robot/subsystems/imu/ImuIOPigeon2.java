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

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import frc.robot.subsystems.drive.PhoenixOdometryThread;
import frc.robot.subsystems.drive.SwerveConstants;
import frc.robot.util.RBSICANBusRegistry;
import java.util.Iterator;
import java.util.Queue;

/** IMU IO for CTRE Pigeon2 (primitive-only hot path) */
public class ImuIOPigeon2 implements ImuIO {

  private static final double DEG_TO_RAD = Math.PI / 180.0;
  private static final double G_TO_MPS2 = 9.80665;

  private final Pigeon2 pigeon =
      new Pigeon2(
          SwerveConstants.kPigeonId, RBSICANBusRegistry.getBus(SwerveConstants.kCANbusName));

  // Cached signals
  private final StatusSignal<Angle> yawSignal = pigeon.getYaw();
  private final StatusSignal<AngularVelocity> yawRateSignal = pigeon.getAngularVelocityZWorld();

  private final StatusSignal<LinearAcceleration> accelX = pigeon.getAccelerationX();
  private final StatusSignal<LinearAcceleration> accelY = pigeon.getAccelerationY();
  private final StatusSignal<LinearAcceleration> accelZ = pigeon.getAccelerationZ();

  private final Queue<Double> odomTimestamps;
  private final Queue<Double> odomYawsDeg;

  // Previous accel for jerk (m/s^2)
  private double prevAx = 0.0, prevAy = 0.0, prevAz = 0.0;
  private long prevTimestampNs = 0L;

  // Reusable buffers for queue-drain (avoid streams)
  private double[] odomTsBuf = new double[8];
  private double[] odomYawRadBuf = new double[8];

  public ImuIOPigeon2() {
    pigeon.getConfigurator().apply(new Pigeon2Configuration());
    pigeon.getConfigurator().setYaw(0.0);

    yawSignal.setUpdateFrequency(SwerveConstants.kOdometryFrequency);
    yawRateSignal.setUpdateFrequency(50.0);

    accelX.setUpdateFrequency(50.0);
    accelY.setUpdateFrequency(50.0);
    accelZ.setUpdateFrequency(50.0);

    pigeon.optimizeBusUtilization();

    odomTimestamps = PhoenixOdometryThread.getInstance().makeTimestampQueue();
    odomYawsDeg = PhoenixOdometryThread.getInstance().registerSignal(yawSignal);
  }

  @Override
  public void updateInputs(ImuIOInputs inputs) {
    final long start = System.nanoTime();

    StatusCode code = BaseStatusSignal.refreshAll(yawSignal, yawRateSignal, accelX, accelY, accelZ);
    inputs.connected = code.isOK();

    // Yaw / rate: Phoenix returns degrees and deg/s here; convert to radians
    final double yawDeg = yawSignal.getValueAsDouble();
    final double yawRateDegPerSec = yawRateSignal.getValueAsDouble();

    inputs.yawPositionRad = yawDeg * DEG_TO_RAD;
    inputs.yawRateRadPerSec = yawRateDegPerSec * DEG_TO_RAD;

    // Accel: Phoenix returns "g" for these signals (common for Pigeon2). Convert to m/s^2
    final double ax = accelX.getValueAsDouble() * G_TO_MPS2;
    final double ay = accelY.getValueAsDouble() * G_TO_MPS2;
    final double az = accelZ.getValueAsDouble() * G_TO_MPS2;

    inputs.linearAccelX = ax;
    inputs.linearAccelY = ay;
    inputs.linearAccelZ = az;

    // Jerk from delta accel / dt
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

    // Drain odometry queues to primitive arrays (timestamps are already doubles; yaws are degrees)
    final int n = drainOdometryQueuesIntoBuffers();
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
    pigeon.setYaw(yawRad / DEG_TO_RAD);
  }

  private int drainOdometryQueuesIntoBuffers() {
    final int nTs = odomTimestamps.size();
    final int nYaw = odomYawsDeg.size();
    final int n = Math.min(nTs, nYaw);
    if (n <= 0) {
      odomTimestamps.clear();
      odomYawsDeg.clear();
      return 0;
    }

    ensureOdomCapacity(n);

    // Iterate without streams (still boxed because Queue<Double>, but avoids stream overhead)
    final Iterator<Double> itT = odomTimestamps.iterator();
    final Iterator<Double> itY = odomYawsDeg.iterator();

    int i = 0;
    while (i < n && itT.hasNext() && itY.hasNext()) {
      odomTsBuf[i] = itT.next();
      odomYawRadBuf[i] = itY.next() * DEG_TO_RAD;
      i++;
    }

    odomTimestamps.clear();
    odomYawsDeg.clear();
    return i;
  }

  private void ensureOdomCapacity(int n) {
    if (odomTsBuf.length >= n) return;
    int newCap = odomTsBuf.length;
    while (newCap < n) newCap *= 2;
    odomTsBuf = new double[newCap];
    odomYawRadBuf = new double[newCap];
  }
}
