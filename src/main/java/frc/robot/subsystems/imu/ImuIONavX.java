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

import static edu.wpi.first.units.Units.*;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.drive.PhoenixOdometryThread;
import frc.robot.subsystems.drive.SwerveConstants;
import java.util.Queue;

/**
 * IMU IO for NavX. Provides yaw, angular velocity, acceleration, odometry, and latency for
 * AdvantageKit logging.
 */
public class ImuIONavX implements ImuIO {

  private final AHRS navx;
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;

  private Translation3d prevAccel = Translation3d.kZero;

  public ImuIONavX() {
    // Initialize NavX over SPI
    navx = new AHRS(NavXComType.kMXP_SPI, (byte) SwerveConstants.kOdometryFrequency);

    // Zero based on alliance
    if (DriverStation.getAlliance().get() == Alliance.Blue) {
      navx.setAngleAdjustment(0.0);
    } else {
      navx.setAngleAdjustment(180.0);
    }
    navx.reset();

    // Register queues for odometry replay/logging
    yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
    yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(navx::getYaw);
  }

  @Override
  public void updateInputs(ImuIOInputs inputs) {
    long start = System.nanoTime();

    inputs.connected = navx.isConnected();
    inputs.yawPosition = Rotation2d.fromDegrees(-navx.getAngle());
    inputs.yawVelocityRadPerSec = RadiansPerSecond.of(-navx.getRawGyroZ());
    inputs.linearAccel =
        new Translation3d(
                navx.getWorldLinearAccelX(),
                navx.getWorldLinearAccelY(),
                navx.getWorldLinearAccelZ())
            .times(9.81); // Convert to m/s^2
    // Compute the jerk and set the new timestamp
    double timediff = (start - inputs.timestampNs) / 1.0e9;
    inputs.jerk = inputs.linearAccel.minus(prevAccel).div(timediff);
    inputs.timestampNs = start;

    // Update odometry history
    inputs.odometryYawTimestamps = yawTimestampQueue.stream().mapToDouble(d -> d).toArray();
    inputs.odometryYawPositions =
        yawPositionQueue.stream().map(d -> Rotation2d.fromDegrees(-d)).toArray(Rotation2d[]::new);

    // Latency in seconds
    long end = System.nanoTime();
    inputs.latencySeconds = (end - start) * 1.0e-9;
  }

  /**
   * Zero the NavX
   *
   * @param yaw The yaw to which to reset the gyro
   */
  @Override
  public void zeroYaw(Rotation2d yaw) {
    navx.setAngleAdjustment(yaw.getDegrees());
    navx.zeroYaw();
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
