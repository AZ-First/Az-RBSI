// Copyright (c) 2024-2026 Az-FIRST
// http://github.com/AZ-First
// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the AdvantageKit-License.md file
// at the root directory of this project.

package frc.robot.subsystems.flywheel_example;

import static frc.robot.Constants.FlywheelConstants.*;
import static frc.robot.Constants.RobotDevices.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drive.SwerveConstants;
import frc.robot.util.SparkUtil;

/**
 * NOTE: To use the Spark Flex / NEO Vortex, replace all instances of "CANSparkMax" with
 * "CANSparkFlex".
 */
public class FlywheelIOSpark implements FlywheelIO {

  // Define the leader / follower motors from the Ports section of RobotContainer
  private final SparkBase leader =
      new SparkMax(FLYWHEEL_LEADER.getDeviceNumber(), MotorType.kBrushless);
  private final SparkBase follower =
      new SparkMax(FLYWHEEL_LEADER.getDeviceNumber(), MotorType.kBrushless);
  private final RelativeEncoder encoder = leader.getEncoder();
  private final SparkClosedLoopController pid = leader.getClosedLoopController();
  // IMPORTANT: Include here all devices listed above that are part of this mechanism!
  public final int[] powerPorts = {
    FLYWHEEL_LEADER.getPowerPort(), FLYWHEEL_FOLLOWER.getPowerPort()
  };

  public FlywheelIOSpark() {

    // Configure leader motor
    var leaderConfig = new SparkFlexConfig();
    leaderConfig
        .idleMode(
            switch (kFlywheelIdleMode) {
              case COAST -> IdleMode.kCoast;
              case BRAKE -> IdleMode.kBrake;
            })
        .smartCurrentLimit((int) SwerveConstants.kDriveCurrentLimit)
        .voltageCompensation(12.0);
    leaderConfig.encoder.uvwMeasurementPeriod(10).uvwAverageDepth(2);
    leaderConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(0.0, 0.0, 0.0)
        .feedForward
        .kV(0.0);
    leaderConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs((int) (1000.0 / SwerveConstants.kOdometryFrequency))
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    SparkUtil.tryUntilOk(
        leader,
        5,
        () ->
            leader.configure(
                leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    SparkUtil.tryUntilOk(leader, 5, () -> encoder.setPosition(0.0));
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    inputs.positionRad = Units.rotationsToRadians(encoder.getPosition() / kFlywheelGearRatio);
    inputs.velocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity() / kFlywheelGearRatio);
    inputs.appliedVolts = leader.getAppliedOutput() * leader.getBusVoltage();
    inputs.currentAmps = new double[] {leader.getOutputCurrent(), follower.getOutputCurrent()};
  }

  @Override
  public void setVoltage(double volts) {
    leader.setVoltage(volts);
  }

  @Override
  public void setVelocity(double velocityRadPerSec, double ffVolts) {
    pid.setSetpoint(
        Units.radiansPerSecondToRotationsPerMinute(velocityRadPerSec) * kFlywheelGearRatio,
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
   * Configure the closed-loop PID
   *
   * <p>TODO: This functionality is no longer supported by the REVLib SparkClosedLoopController
   * class. In order to keep control of the flywheel's underlying funtionality, shift everything to
   * SmartMotion control.
   */
  @Override
  public void configurePID(double kP, double kI, double kD) {
    // pid.setP(kP, 0);
    // pid.setI(kI, 0);
    // pid.setD(kD, 0);
    // pid.setFF(0, 0);
  }
}
