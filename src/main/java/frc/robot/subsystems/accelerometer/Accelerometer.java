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
 */
public class Accelerometer extends VirtualSubsystem {

  // Gravitational acceleration
  private static final double G_TO_MPS2 = 9.80665;

  // Define hardware interfaces
  private final RioAccelIO rio;
  private final RioAccelIO.Inputs rioInputs = new RioAccelIO.Inputs();
  private final Imu imu;

  // Variables needed during the periodic
  private Translation3d rawRio, rioAcc, rioJerk, imuAcc, imuJerk;
  private Translation3d prevRioAcc = Translation3d.kZero;

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
    rawRio = new Translation3d(rioInputs.xG, rioInputs.yG, rioInputs.zG);
    rioAcc = rawRio.rotateBy(RobotConstants.kRioOrientation).times(G_TO_MPS2);

    // Acceleration from previous loop
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
      rioJerk = rioAcc.minus(prevRioAcc).div(Constants.loopPeriodSecs);
      imuJerk = imuInputs.linearJerk.rotateBy(RobotConstants.kIMUOrientation);
      Logger.recordOutput("Accel/Rio/Jerk_mps3", rioJerk);
      Logger.recordOutput("Accel/IMU/Jerk_mps3", imuJerk);

      final double[] ts = imuInputs.odometryYawTimestamps;
      if (ts.length > 0) {
        Logger.recordOutput("IMU/OdometryLatencySec", Timer.getFPGATimestamp() - ts[ts.length - 1]);
      }
    }
  }
}
