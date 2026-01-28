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
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.subsystems.imu.Imu;
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
  private final Imu imu;

  // RIO and IMU rotations with respect to the robot
  private static final Rotation3d kRioRot = new Rotation3d(0, 0, kRioOrientation.getRadians());
  private static final Rotation3d kImuRot = new Rotation3d(0, 0, kIMUOrientation.getRadians());

  private Translation3d prevRioAccel = Translation3d.kZero;

  public Accelerometer(Imu imu) {
    this.imu = imu;
  }

  @Override
  public void rbsiPeriodic() {
    long t0 = System.nanoTime();
    // --- Updated IMU readings ---
    final var imuInputs = imu.getInputs();

    // --- Apply orientation corrections ---
    Translation3d rioAccVector =
        new Translation3d(rioAccel.getX(), rioAccel.getY(), rioAccel.getZ())
            .rotateBy(kRioRot)
            .times(9.81); // convert to m/s^2

    Translation3d imuAccVector =
        imuInputs
            .linearAccel
            .rotateBy(kImuRot)
            .times(1.00); // already converted to m/s^2 in ImuIO implementation

    // --- Compute jerks ---
    Translation3d rioJerk = rioAccVector.minus(prevRioAccel).div(Constants.loopPeriodSecs);
    Translation3d imuJerk = imuInputs.jerk.rotateBy(kImuRot);

    // --- Log to AdvantageKit ---
    Logger.recordOutput("Accel/Rio/Accel_mps2", rioAccVector);
    Logger.recordOutput("Accel/Rio/Jerk_mps3", rioJerk);
    Logger.recordOutput("Accel/IMU/Accel_mps2", imuAccVector);
    Logger.recordOutput("Accel/IMU/Jerk_mps3", imuJerk);

    // --- Log IMU latency ---
    final double[] ts = imuInputs.odometryYawTimestamps;
    if (ts.length > 0) {
      double latencySeconds = Timer.getFPGATimestamp() - ts[ts.length - 1];
      Logger.recordOutput("IMU/OdometryLatencySec", latencySeconds);
    }

    prevRioAccel = rioAccVector;

    long t1 = System.nanoTime();
    Logger.recordOutput("Loop/Accel/total_ms", (t1 - t0) / 1e6);
  }
}
