// Copyright (c) 2024-2027 Az-FIRST
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

import frc.robot.Constants.RobotConstants;
import frc.robot.subsystems.imu.Imu;
import frc.robot.util.VirtualSubsystem;
import org.littletonrobotics.junction.Logger;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.system.Timer;

/**
 * Accelerometer subsystem (VirtualSubsystem)
 *
 * <p>Reads the Systemcore's onboard accelerometer and the swerve IMU, rotates both into the robot
 * frame, and logs acceleration and jerk through AdvantageKit.
 */
public class Accelerometer extends VirtualSubsystem {

  // Define hardware interfaces
  private final ControllerAccelIO controller;
  private final ControllerAccelIOInputsAutoLogged controllerInputs =
      new ControllerAccelIOInputsAutoLogged();
  private final Imu imu;

  private Translation3d prevControllerAcc;
  private long prevControllerTimestampNs = Long.MIN_VALUE;

  // Log decimation
  private int loopCount = 0;
  private static final int LOG_EVERY_N = 5; // 10Hz for heavier logs

  public Accelerometer(Imu imu) {
    this(imu, new ControllerAccelIOOnboard());
  }

  public Accelerometer(Imu imu, ControllerAccelIO controller) {
    this.imu = imu;
    this.controller = controller;
  }

  // Priority value for this virtual subsystem
  @Override
  protected int getPeriodPriority() {
    return +10;
  }

  @Override
  public void rbsiPeriodic() {
    final var imuInputs = imu.getInputs();
    controller.updateInputs(controllerInputs);
    Logger.processInputs("Accel/Systemcore", controllerInputs);

    // OnboardIMU returns m/s^2; only rotate the raw sensor axes into the robot frame.
    Translation3d controllerAcc =
        new Translation3d(
                controllerInputs.xMetersPerSecondSq,
                controllerInputs.yMetersPerSecondSq,
                controllerInputs.zMetersPerSecondSq)
            .rotateBy(RobotConstants.kSystemcoreOrientation);

    Translation3d controllerJerk = Translation3d.ZERO;
    if (controllerInputs.connected
        && prevControllerAcc != null
        && controllerInputs.timestampNs > prevControllerTimestampNs) {
      double dtSecs = (controllerInputs.timestampNs - prevControllerTimestampNs) * 1e-9;
      controllerJerk = controllerAcc.minus(prevControllerAcc).div(dtSecs);
    }
    prevControllerAcc = controllerInputs.connected ? controllerAcc : null;
    prevControllerTimestampNs = controllerInputs.timestampNs;

    // IMU accelerations and jerks
    Translation3d imuAcc = imuInputs.linearAccel.rotateBy(RobotConstants.kIMUOrientation);

    // Logging
    Logger.recordOutput("Accel/Systemcore/Accel_mps2", controllerAcc);
    Logger.recordOutput("Accel/IMU/Accel_mps2", imuAcc);

    // Every N loops, compute and log the Jerk
    final boolean doHeavyLogs = (++loopCount >= LOG_EVERY_N);
    if (doHeavyLogs) {
      loopCount = 0;
      Translation3d imuJerk = imuInputs.linearJerk.rotateBy(RobotConstants.kIMUOrientation);
      Logger.recordOutput("Accel/Systemcore/Jerk_mps3", controllerJerk);
      Logger.recordOutput("Accel/IMU/Jerk_mps3", imuJerk);

      final double[] ts = imuInputs.odometryYawTimestamps;
      if (ts.length > 0) {
        Logger.recordOutput("Odometry/IMULatencySec", Timer.getTimestamp() - ts[ts.length - 1]);
      }
    }
  }
}
