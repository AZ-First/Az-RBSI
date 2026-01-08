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

import static frc.robot.Constants.RobotConstants.*;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import frc.robot.Constants;
import frc.robot.subsystems.imu.ImuIO;
import frc.robot.util.VirtualSubsystem;
import org.littletonrobotics.junction.Logger;

/**
 * Accelerometer subsystem (built upon a virtual subsystem)
 *
 * <p>This virtual subsystem pulls the acceleration values from both the RoboRIO and the swerve's
 * IMU (either Pigeon2 or NavX) and logs them to both AdvantageKit and the SmartDashboard. In
 * addition to the accelerations, the jerk (a-dot or x-tripple-dot) is computed from the delta
 * accelerations.
 */
public class Accelerometer extends VirtualSubsystem {

  private final BuiltInAccelerometer rioAccel = new BuiltInAccelerometer();
  private final ImuIO imuIO;
  private final ImuIO.ImuIOInputs imuInputs = new ImuIO.ImuIOInputs();

  private Translation3d prevRioAccel = Translation3d.kZero;

  public Accelerometer(ImuIO imuIO) {
    this.imuIO = imuIO;
  }

  @Override
  public void periodic() {
    // --- Update IMU readings ---
    imuIO.updateInputs(imuInputs);

    // --- Apply orientation corrections ---
    Translation3d rioAccVector =
        new Translation3d(rioAccel.getX(), rioAccel.getY(), rioAccel.getZ())
            .rotateBy(new Rotation3d(0., 0., kRioOrientation.getRadians()))
            .times(9.81); // convert to m/s^2

    Translation3d imuAccVector =
        imuInputs
            .linearAccel
            .rotateBy(new Rotation3d(0., 0., kIMUOrientation.getRadians()))
            .times(1.00); // already converted to m/s^2 in ImuIO implementation

    // --- Compute jerks ---
    Translation3d rioJerk = rioAccVector.minus(prevRioAccel).div(Constants.loopPeriodSecs);
    Translation3d imuJerk =
        imuInputs.jerk.rotateBy(new Rotation3d(0.0, 0.0, kIMUOrientation.getRadians()));

    // --- Log to AdvantageKit ---
    Logger.recordOutput("Accel/Rio/Accel_mps2", rioAccVector);
    Logger.recordOutput("Accel/Rio/Jerk_mps3", rioJerk);
    Logger.recordOutput("Accel/IMU/Accel_mps2", imuAccVector);
    Logger.recordOutput("Accel/IMU/Jerk_mps3", imuJerk);

    // --- Log IMU latency ---
    if (imuInputs.odometryYawTimestamps.length > 0) {
      double latencySeconds =
          System.currentTimeMillis() / 1000.0
              - imuInputs.odometryYawTimestamps[imuInputs.odometryYawTimestamps.length - 1];
      Logger.recordOutput("IMU/LatencySec", latencySeconds);
    }

    prevRioAccel = rioAccVector;
  }
}
