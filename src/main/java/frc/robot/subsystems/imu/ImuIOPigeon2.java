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

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.subsystems.drive.PhoenixOdometryThread;
import frc.robot.subsystems.drive.SwerveConstants;
import java.util.Queue;

/** IMU IO for CTRE Pigeon2 */
public class ImuIOPigeon2 implements ImuIO {

  private final Pigeon2 pigeon;
  private final StatusSignal<Angle> yawSignal;
  private final StatusSignal<AngularVelocity> yawRateSignal;
  private final Queue<Double> odomTimestamps;
  private final Queue<Double> odomYaws;

  private Translation3d prevAccel = Translation3d.kZero;

  /** Constructor */
  public ImuIOPigeon2() {
    pigeon = new Pigeon2(SwerveConstants.kPigeonId, SwerveConstants.kCANBus);
    pigeon.getConfigurator().apply(new Pigeon2Configuration());
    pigeon.optimizeBusUtilization();

    // Get the Signals
    this.yawSignal = pigeon.getYaw();
    this.yawRateSignal = pigeon.getAngularVelocityZWorld();

    // Create queues for odometry logging/replay inside the class
    odomTimestamps = PhoenixOdometryThread.getInstance().makeTimestampQueue();
    odomYaws =
        PhoenixOdometryThread.getInstance().registerSignal(() -> yawSignal.getValueAsDouble());

    // Configure update rates
    yawSignal.setUpdateFrequency(250);
    yawRateSignal.setUpdateFrequency(50);
  }

  /** Update inputs for logging and robot code */
  @Override
  public void updateInputs(ImuIOInputs inputs) {
    long start = System.nanoTime();

    inputs.connected = BaseStatusSignal.refreshAll(yawSignal, yawRateSignal).equals(StatusCode.OK);
    inputs.yawPosition = Rotation2d.fromDegrees(yawSignal.getValueAsDouble());
    inputs.yawVelocityRadPerSec = RadiansPerSecond.of(yawRateSignal.getValueAsDouble());
    inputs.linearAccel =
        new Translation3d(
            pigeon.getAccelerationX().getValueAsDouble(),
            pigeon.getAccelerationY().getValueAsDouble(),
            pigeon.getAccelerationZ().getValueAsDouble());
    // Compute the jerk and set the new timestamp
    double timediff = (start - inputs.timestampNs) / 1.0e9;
    inputs.jerk = inputs.linearAccel.minus(prevAccel).div(timediff);
    inputs.timestampNs = start;

    // Update odometry history
    double now = System.currentTimeMillis() / 1000.0;
    odomTimestamps.add(now);
    odomYaws.add(inputs.yawPosition.getDegrees());

    while (odomTimestamps.size() > 50) odomTimestamps.poll();
    while (odomYaws.size() > 50) odomYaws.poll();

    inputs.odometryYawTimestamps = odomTimestamps.stream().mapToDouble(d -> d).toArray();
    inputs.odometryYawPositions =
        odomYaws.stream().map(d -> Rotation2d.fromDegrees(d)).toArray(Rotation2d[]::new);

    // Latency in seconds
    long end = System.nanoTime();
    inputs.latencySeconds = (end - start) * 1.0e-9;
  }

  /**
   * Zero the Pigeon2
   *
   * @param yaw The yaw to which to reset the gyro
   */
  @Override
  public void zeroYaw(Rotation2d yaw) {
    pigeon.setYaw(yaw.getDegrees());
  }

  // /**
  //  * Zero the Pigeon2
  //  *
  //  * <p>This method should always rezero the pigeon in ALWAYS-BLUE-ORIGIN orientation. Testing,
  //  * however, shows that it's not doing what I think it should be doing. There is likely
  //  * interference with something else in the odometry
  //  */
  // @Override
  // public void zero() {
  //   // With the Pigeon facing forward, forward depends on the alliance selected.
  //   if (DriverStation.getAlliance().get() == Alliance.Blue) {
  //     System.out.println("Alliance Blue: Setting YAW to 0");
  //     pigeon.setYaw(0.0);
  //   } else {
  //     System.out.println("Alliance Red: Setting YAW to 180");
  //     pigeon.setYaw(180.0);
  //   }
  // }

}
