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

import com.ctre.phoenix6.hardware.Pigeon2;
import com.studica.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.util.VirtualSubsystem;
import frc.robot.util.YagslConstants;
import java.util.function.Supplier;
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

  private final BuiltInAccelerometer rioAccelerometer = new BuiltInAccelerometer();
  private final Pigeon2 pigeonAccelerometer;
  private final AHRS navXAccelerometer;

  // Define the 3D vectors needed to hold values
  private Translation3d rioAccVector;
  private Translation3d imuAccVector;
  private Translation3d prevRioAccel = Translation3d.kZero;
  private Translation3d prevImuAccel = Translation3d.kZero;
  private Translation3d rioJerkVector;
  private Translation3d imuJerkVector;

  /** Constructor method, takes the IMU accelerometer supplier */
  public Accelerometer(Supplier<Object> accelerometerSupplier) {

    // Get the object from the supplier
    Object accelerometer = accelerometerSupplier.get();

    // Case out the type of accelerometer
    switch (Constants.getSwerveType()) {
      case PHOENIX6:
        // CTRE Tuner X drive bases must use a Pigeon2
        pigeonAccelerometer = (Pigeon2) accelerometer;
        navXAccelerometer = null;
        break;

      case YAGSL:
        // Logic checking the type of IMU included in the parsed YAGSL:
        if (YagslConstants.swerveDriveJson.imu.type == "pigeon2") {
          pigeonAccelerometer = (Pigeon2) accelerometer;
          navXAccelerometer = null;
          break;
        }

        if (YagslConstants.swerveDriveJson.imu.type == "navx"
            || YagslConstants.swerveDriveJson.imu.type == "navx_spi") {
          navXAccelerometer = (AHRS) accelerometer;
          pigeonAccelerometer = null;
          break;
        }

      default:
        // Otherwise kick a message to the console, and set to null
        System.out.println("WARNING: Cannot initialize IMU's accelerometer for logging!");
        this.pigeonAccelerometer = null;
        this.navXAccelerometer = null;
    }
  }

  /** Get accelerations, compute jerks, log everything */
  public void periodic() {

    // Log the execution time
    long start = System.nanoTime();

    // Compute the Rio's acceleration, rotated as needed in the XY plane (yaw)
    // RoboRio provides accelerations in `g`, from -8 to +8; convert to m/s^2
    rioAccVector =
        new Translation3d(rioAccelerometer.getX(), rioAccelerometer.getY(), rioAccelerometer.getZ())
            .rotateBy(new Rotation3d(0., 0., kRioOrientation.getRadians()))
            .times(9.81);

    // Compute the IMU's acceleration, rotated as needed
    if (pigeonAccelerometer != null) {
      // Pigeon provides accelerations in `g`, from -2 to +2; convert to m/s^2
      imuAccVector =
          new Translation3d(
              pigeonAccelerometer.getAccelerationX().getValueAsDouble(),
              pigeonAccelerometer.getAccelerationY().getValueAsDouble(),
              pigeonAccelerometer.getAccelerationZ().getValueAsDouble());
    } else if (navXAccelerometer != null) {
      // NavX provides accelerations in ...
      imuAccVector =
          new Translation3d(
              navXAccelerometer.getWorldLinearAccelX(),
              navXAccelerometer.getWorldLinearAccelY(),
              navXAccelerometer.getWorldLinearAccelZ());
    } else {
      imuAccVector = Translation3d.kZero;
    }
    imuAccVector =
        imuAccVector.rotateBy(new Rotation3d(0., 0., kIMUOrientation.getRadians())).times(9.81);

    // Compute the jerks ((current - prev accel) / loop time)
    rioJerkVector = rioAccVector.minus(prevRioAccel).div(Constants.loopPeriodSecs);
    imuJerkVector = imuAccVector.minus(prevImuAccel).div(Constants.loopPeriodSecs);

    // Log everything to both AdvantageKit and SmartDashboard
    Logger.recordOutput("Acceleration/Rio/Accel_mss", rioAccVector);
    Logger.recordOutput("Acceleration/Rio/Jerk_msss", rioJerkVector);
    Logger.recordOutput("Acceleration/IMU/Accel_mss", imuAccVector);
    Logger.recordOutput("Acceleration/IMU/Jerk_msss", imuJerkVector);
    SmartDashboard.putNumber("RioXAccel", rioAccVector.getX());
    SmartDashboard.putNumber("RioYAccel", rioAccVector.getY());
    SmartDashboard.putNumber("IMUXAccel", imuAccVector.getX());
    SmartDashboard.putNumber("IMUYAccel", imuAccVector.getY());

    // Set the "previous" accelerations to the current for the next loop
    prevRioAccel = rioAccVector;
    prevImuAccel = imuAccVector;

    // Quick logging to see how long this periodic takes
    long finish = System.nanoTime();
    long timeElapsed = finish - start;
    Logger.recordOutput("LoggedRobot/AccelCodeMS", (double) timeElapsed / 1.e6);
  }
}
