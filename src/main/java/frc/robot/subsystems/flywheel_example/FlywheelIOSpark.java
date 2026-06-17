// Copyright (c) 2024-2026 Az-FIRST
// http://github.com/AZ-First
// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
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

package frc.robot.subsystems.flywheel_example;

import static frc.robot.Constants.FlywheelConstants.*;
import static frc.robot.Constants.RobotDevices.*;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.subsystems.drive.SwerveConstants;
import frc.robot.util.SparkUtil;
import org.littletonrobotics.junction.Logger;

/**
 * NOTE: To use the Spark Flex / NEO Vortex, replace all instances of "CANSparkMax" with
 * "CANSparkFlex".
 */
public class FlywheelIOSpark implements FlywheelIO {

  // Define the leader / follower motors from the RobotDevices section of RobotContainer
  private final SparkMax leader =
      new SparkMax(FLYWHEEL_LEADER.getDeviceNumber(), MotorType.kBrushless);
  private final SparkMax follower =
      new SparkMax(FLYWHEEL_FOLLOWER.getDeviceNumber(), MotorType.kBrushless);
  private final RelativeEncoder encoder = leader.getEncoder();
  private final SparkClosedLoopController pid = leader.getClosedLoopController();
  // IMPORTANT: Include here all devices listed above that are part of this mechanism!
  public final int[] powerPorts = {
    FLYWHEEL_LEADER.getPowerPort(), FLYWHEEL_FOLLOWER.getPowerPort()
  };
  private final SparkMaxConfig leaderConfig = new SparkMaxConfig();
  private SimpleMotorFeedforward ff = new SimpleMotorFeedforward(kRealS, kRealV, kRealA);

  @Override
  public int[] powerPorts() {
    return powerPorts;
  }

  public FlywheelIOSpark() {

    // Configure leader motor
    leaderConfig
        .idleMode(
            switch (kIdleMode) {
              case COAST -> IdleMode.kCoast;
              case BRAKE -> IdleMode.kBrake;
            })
        .smartCurrentLimit((int) SwerveConstants.kDriveCurrentLimit)
        .voltageCompensation(DrivebaseConstants.kNominalVoltage);
    leaderConfig.encoder.uvwMeasurementPeriod(10).uvwAverageDepth(2);
    leaderConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(kRealP, 0.0, kRealD)
        .feedForward
        .kS(kRealS)
        .kV(kRealV)
        .kA(kRealA);
    leaderConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs((int) (1000.0 / SwerveConstants.kOdometryFrequency))
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs((int) (Constants.kLoopPeriodSecs * 1000.))
        .appliedOutputPeriodMs((int) (Constants.kLoopPeriodSecs * 1000.))
        .busVoltagePeriodMs((int) (Constants.kLoopPeriodSecs * 1000.))
        .outputCurrentPeriodMs((int) (Constants.kLoopPeriodSecs * 1000.));
    leaderConfig
        .openLoopRampRate(DrivebaseConstants.kDriveOpenLoopRampPeriodSecs)
        .closedLoopRampRate(DrivebaseConstants.kDriveClosedLoopRampPeriodSecs);
    SparkUtil.tryUntilOk(
        leader,
        5,
        () ->
            leader.configure(
                leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    var followerConfig = new SparkMaxConfig();
    followerConfig.follow(leader);
    SparkUtil.tryUntilOk(
        follower,
        5,
        () ->
            follower.configure(
                followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    SparkUtil.tryUntilOk(leader, 5, () -> encoder.setPosition(0.0));
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    inputs.positionRad = Units.rotationsToRadians(encoder.getPosition() / kGearRatio);
    inputs.velocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity() / kGearRatio);
    inputs.appliedVolts = leader.getAppliedOutput() * leader.getBusVoltage();
    inputs.currentAmps = new double[] {leader.getOutputCurrent(), follower.getOutputCurrent()};

    // AdvantageKit logging
    Logger.recordOutput("Flywheel/PositionRad", inputs.positionRad);
    Logger.recordOutput("Flywheel/VelocityRadPerSec", inputs.velocityRadPerSec);
    Logger.recordOutput("Flywheel/AppliedVolts", inputs.appliedVolts);
    Logger.recordOutput("Flywheel/LeaderCurrent", inputs.currentAmps[0]);
    Logger.recordOutput("Flywheel/FollowerCurrent", inputs.currentAmps[1]);
  }

  @Override
  public void setVoltage(double volts) {
    leader.setVoltage(volts);
  }

  @Override
  public void setPercent(double percent) {
    leader.set(percent);
  }

  @Override
  public void setVelocity(double velocityRadPerSec) {
    double ffVolts = ff.calculate(velocityRadPerSec);
    pid.setSetpoint(
        Units.radiansPerSecondToRotationsPerMinute(velocityRadPerSec) * kGearRatio,
        ControlType.kVelocity,
        ClosedLoopSlot.kSlot0,
        ffVolts,
        ArbFFUnits.kVoltage);
  }

  @Override
  public void stop() {
    leader.stopMotor();
  }

  /**
   * Configure the closed-loop control gains
   *
   * <p>REVLib 2026 applies closed-loop gains through the Spark configuration object rather than
   * direct setters on {@link SparkClosedLoopController}.
   */
  @Override
  public void configureGains(double kP, double kI, double kD, double kS, double kV) {
    configureGains(kP, kI, kD, kS, kV, 0.0);
  }

  @Override
  public void configureGains(double kP, double kI, double kD, double kS, double kV, double kA) {
    ff = new SimpleMotorFeedforward(kS, kV, kA);
    leaderConfig.closedLoop.pid(kP, kI, kD).feedForward.kS(kS).kV(kV).kA(kA);
    SparkUtil.tryUntilOk(
        leader,
        5,
        () ->
            leader.configure(
                leaderConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters));
  }
}
