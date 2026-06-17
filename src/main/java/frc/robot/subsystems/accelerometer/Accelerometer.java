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

import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.SensorConstants;
import frc.robot.subsystems.imu.Imu;
import frc.robot.util.TimeUtil;
import frc.robot.util.VirtualSubsystem;
import org.littletonrobotics.junction.Logger;

/**
 * Accelerometer subsystem (VirtualSubsystem)
 *
 * <p>This virtual subsystem pulls the acceleration values from both the RoboRIO and the swerve's
 * IMU (either Pigeon2 or NavX) and logs them to both AdvantageKitd. In addition to the
 * accelerations, the jerk (a-dot or x-tripple-dot) is computed from the delta accelerations.
 */
public class Accelerometer extends VirtualSubsystem {

  // Define hardware interfaces
  private final RioAccelIO rio;
  private final RioAccelIO.Inputs rioInputs = new RioAccelIO.Inputs();
  private final Imu imu;

  // Variables needed during the periodic
  private Translation3d rawRio, rioAcc, rioJerk, imuAcc, imuJerk;
  private Translation3d prevRioAcc = null;

  // Log decimation
  private int loopCount = 0;
  private static final int LOG_EVERY_N = 5; // 10Hz for heavier logs

  // Profiling decimation
  private int profileCount = 0;
  private static final int PROFILE_EVERY_N = 50; // 1Hz profiling

  public Accelerometer(Imu imu) {
    this(imu, new RioAccelIORoboRIO(SensorConstants.kRioAccelerometerSampleRateHz));
  }

  public Accelerometer(Imu imu, RioAccelIO rio) {
    this.imu = imu;
    this.rio = rio;
  }

  // Priority value for this virtual subsystem
  @Override
  protected int getPeriodPriority() {
    return +10;
  }

  @Override
  public void rbsiPeriodic() {
    final boolean doProfile = (++profileCount >= PROFILE_EVERY_N);
    if (doProfile) profileCount = 0;

    // Fetch the values from the IMU and the RIO
    final var imuInputs = imu.getInputs(); // should be primitive ImuIOInputs
    rio.updateInputs(rioInputs);

    // Compute RIO accelerations and jerks
    rawRio =
        new Translation3d(
            rioInputs.xG * Constants.kGravityMetersPerSecSq,
            rioInputs.yG * Constants.kGravityMetersPerSecSq,
            rioInputs.zG * Constants.kGravityMetersPerSecSq);
    rioAcc = rawRio.rotateBy(RobotConstants.kRioOrientation);

    Translation3d rioJerkThisLoop =
        prevRioAcc == null
            ? Translation3d.kZero
            : rioAcc.minus(prevRioAcc).div(Constants.kLoopPeriodSecs);
    prevRioAcc = rioAcc;

    // IMU accelerations and jerks
    imuAcc = imuInputs.linearAccel.rotateBy(RobotConstants.kIMUOrientation);

    // Logging
    Logger.recordOutput("Accel/Rio/Accel_mps2", rioAcc);
    Logger.recordOutput("Accel/IMU/Accel_mps2", imuAcc);

    // Every N loops, compute and log the Jerk
    final boolean doHeavyLogs = (++loopCount >= LOG_EVERY_N);
    if (doHeavyLogs) {
      loopCount = 0;
      rioJerk = rioJerkThisLoop;
      imuJerk = imuInputs.linearJerk.rotateBy(RobotConstants.kIMUOrientation);
      Logger.recordOutput("Accel/Rio/Jerk_mps3", rioJerk);
      Logger.recordOutput("Accel/IMU/Jerk_mps3", imuJerk);

      final double[] ts = imuInputs.odometryYawTimestamps;
      if (ts.length > 0) {
        Logger.recordOutput("Odometry/IMULatencySec", TimeUtil.now() - ts[ts.length - 1]);
      }
    }
  }
}
