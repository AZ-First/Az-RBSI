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

package frc.robot.subsystems.accelerometer;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Constants.RobotConstants;
import frc.robot.subsystems.imu.Imu;
import frc.robot.util.VirtualSubsystem;
import org.littletonrobotics.junction.Logger;

/**
 * Accelerometer subsystem (VirtualSubsystem)
 *
 * <p>This virtual subsystem pulls the acceleration values from both the RoboRIO and the swerve's
 * IMU (either Pigeon2 or NavX) and logs them to both AdvantageKitd. In addition to the
 * accelerations, the jerk (a-dot or x-tripple-dot) is computed from the delta accelerations.
 *
 * <p>Primitive-only hot path: no WPILib geometry objects or Units objects.
 */
public class Accelerometer extends VirtualSubsystem {
  private static final double G_TO_MPS2 = 9.80665;

  private final RioAccelIO rio;
  private final RioAccelIO.Inputs rioInputs = new RioAccelIO.Inputs();
  private final Imu imu;

  // Precompute yaw-only rotation terms
  private static final double rioCos = Math.cos(RobotConstants.kRioOrientation.getRadians());
  private static final double rioSin = Math.sin(RobotConstants.kRioOrientation.getRadians());
  private static final double imuCos = Math.cos(RobotConstants.kIMUOrientation.getRadians());
  private static final double imuSin = Math.sin(RobotConstants.kIMUOrientation.getRadians());

  // Previous Rio accel (m/s^2)
  private double prevRioAx = 0.0, prevRioAy = 0.0, prevRioAz = 0.0;

  // Reusable arrays for logging
  private final double[] rioAccelArr = new double[3];
  private final double[] rioJerkArr = new double[3];
  private final double[] imuAccelArr = new double[3];
  private final double[] imuJerkArr = new double[3];

  // Log decimation
  private int loopCount = 0;
  private static final int LOG_EVERY_N = 5; // 10Hz for heavier logs

  // Profiling decimation
  private int profileCount = 0;
  private static final int PROFILE_EVERY_N = 50; // 1Hz profiling

  public Accelerometer(Imu imu) {
    this.imu = imu;
    this.rio = new RioAccelIORoboRIO(200.0); // 200 Hz is a good start
  }

  @Override
  public void rbsiPeriodic() {
    final boolean doProfile = (++profileCount >= PROFILE_EVERY_N);
    if (doProfile) profileCount = 0;

    // Fetch the values from the IMU and the RIO
    final var imuInputs = imu.getInputs(); // should be primitive ImuIOInputs
    rio.updateInputs(rioInputs);

    // Compute RIO accelerations and jerks
    final double rawX = rioInputs.xG;
    final double rawY = rioInputs.yG;
    final double rawZ = rioInputs.zG;

    final double rioAx = (rioCos * rawX - rioSin * rawY) * G_TO_MPS2;
    final double rioAy = (rioSin * rawX + rioCos * rawY) * G_TO_MPS2;
    final double rioAz = rawZ * G_TO_MPS2;

    final double rioJx = (rioAx - prevRioAx) / Constants.loopPeriodSecs;
    final double rioJy = (rioAy - prevRioAy) / Constants.loopPeriodSecs;
    final double rioJz = (rioAz - prevRioAz) / Constants.loopPeriodSecs;

    // Acceleration from previous loop
    prevRioAx = rioAx;
    prevRioAy = rioAy;
    prevRioAz = rioAz;

    // IMU rotate is also compute-only (already primitives)
    final double imuAx = (imuCos * imuInputs.linearAccelX - imuSin * imuInputs.linearAccelY);
    final double imuAy = (imuSin * imuInputs.linearAccelX + imuCos * imuInputs.linearAccelY);
    final double imuAz = imuInputs.linearAccelZ;

    final double imuJx = (imuCos * imuInputs.jerkX - imuSin * imuInputs.jerkY);
    final double imuJy = (imuSin * imuInputs.jerkX + imuCos * imuInputs.jerkY);
    final double imuJz = imuInputs.jerkZ;

    // Fill accel arrays (still math)
    rioAccelArr[0] = rioAx;
    rioAccelArr[1] = rioAy;
    rioAccelArr[2] = rioAz;
    imuAccelArr[0] = imuAx;
    imuAccelArr[1] = imuAy;
    imuAccelArr[2] = imuAz;

    final boolean doHeavyLogs = (++loopCount >= LOG_EVERY_N);
    if (doHeavyLogs) {
      loopCount = 0;
      rioJerkArr[0] = rioJx;
      rioJerkArr[1] = rioJy;
      rioJerkArr[2] = rioJz;
      imuJerkArr[0] = imuJx;
      imuJerkArr[1] = imuJy;
      imuJerkArr[2] = imuJz;
    }

    // Logging
    Logger.recordOutput("Accel/Rio/Accel_mps2", rioAccelArr);
    Logger.recordOutput("Accel/IMU/Accel_mps2", imuAccelArr);

    if (doHeavyLogs) {
      Logger.recordOutput("Accel/Rio/Jerk_mps3", rioJerkArr);
      Logger.recordOutput("Accel/IMU/Jerk_mps3", imuJerkArr);

      final double[] ts = imuInputs.odometryYawTimestamps;
      if (ts.length > 0) {
        Logger.recordOutput("IMU/OdometryLatencySec", Timer.getFPGATimestamp() - ts[ts.length - 1]);
      }
    }
  }
}
