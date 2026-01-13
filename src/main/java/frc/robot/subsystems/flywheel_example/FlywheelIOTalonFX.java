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

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.PhoenixUtil;

public class FlywheelIOTalonFX implements FlywheelIO {

  // Define the leader / follower motors from the Ports section of RobotContainer
  private final TalonFX leader =
      new TalonFX(FLYWHEEL_LEADER.getDeviceNumber(), FLYWHEEL_LEADER.getCANBus());
  private final TalonFX follower =
      new TalonFX(FLYWHEEL_FOLLOWER.getDeviceNumber(), FLYWHEEL_FOLLOWER.getCANBus());
  // IMPORTANT: Include here all devices listed above that are part of this mechanism!
  public final int[] powerPorts = {
    FLYWHEEL_LEADER.getPowerPort(), FLYWHEEL_FOLLOWER.getPowerPort()
  };

  private final StatusSignal<Angle> leaderPosition = leader.getPosition();
  private final StatusSignal<AngularVelocity> leaderVelocity = leader.getVelocity();
  private final StatusSignal<Voltage> leaderAppliedVolts = leader.getMotorVoltage();
  private final StatusSignal<Current> leaderCurrent = leader.getSupplyCurrent();
  private final StatusSignal<Current> followerCurrent = follower.getSupplyCurrent();

  private final TalonFXConfiguration config = new TalonFXConfiguration();

  public FlywheelIOTalonFX() {
    config.CurrentLimits.SupplyCurrentLimit = 30.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode =
        switch (kFlywheelIdleMode) {
          case COAST -> NeutralModeValue.Coast;
          case BRAKE -> NeutralModeValue.Brake;
        };
    // Build the OpenLoopRampsConfigs and ClosedLoopRampsConfigs for current smoothing
    OpenLoopRampsConfigs openRamps = new OpenLoopRampsConfigs();
    openRamps.DutyCycleOpenLoopRampPeriod = kFlywheelOpenLoopRampPeriod;
    openRamps.VoltageOpenLoopRampPeriod = kFlywheelOpenLoopRampPeriod;
    openRamps.TorqueOpenLoopRampPeriod = kFlywheelOpenLoopRampPeriod;
    ClosedLoopRampsConfigs closedRamps = new ClosedLoopRampsConfigs();
    closedRamps.DutyCycleClosedLoopRampPeriod = kFlywheelClosedLoopRampPeriod;
    closedRamps.VoltageClosedLoopRampPeriod = kFlywheelClosedLoopRampPeriod;
    closedRamps.TorqueClosedLoopRampPeriod = kFlywheelClosedLoopRampPeriod;
    // Apply the open- and closed-loop ramp configuration for current smoothing
    config.withClosedLoopRamps(closedRamps).withOpenLoopRamps(openRamps);

    // Apply the configurations to the flywheel motors
    leader.getConfigurator().apply(config);
    follower.getConfigurator().apply(config);
    // If follower rotates in the opposite direction, set "MotorAlignmentValue" to Opposed
    follower.setControl(new Follower(leader.getDeviceID(), MotorAlignmentValue.Aligned));

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, leaderPosition, leaderVelocity, leaderAppliedVolts, leaderCurrent, followerCurrent);
    leader.optimizeBusUtilization();
    follower.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        leaderPosition, leaderVelocity, leaderAppliedVolts, leaderCurrent, followerCurrent);
    inputs.positionRad =
        Units.rotationsToRadians(leaderPosition.getValueAsDouble()) / kFlywheelGearRatio;
    inputs.velocityRadPerSec =
        Units.rotationsToRadians(leaderVelocity.getValueAsDouble()) / kFlywheelGearRatio;
    inputs.appliedVolts = leaderAppliedVolts.getValueAsDouble();
    inputs.currentAmps =
        new double[] {leaderCurrent.getValueAsDouble(), followerCurrent.getValueAsDouble()};
  }

  @Override
  public void setVoltage(double volts) {
    leader.setControl(new VoltageOut(volts));
  }

  @Override
  public void setVelocity(double velocityRadPerSec, double ffVolts) {
    leader.setControl(new VelocityVoltage(Units.radiansToRotations(velocityRadPerSec)));
  }

  @Override
  public void stop() {
    leader.stopMotor();
  }

  /**
   * Set the PID portion of the Slot0 closed-loop configuration
   *
   * @param kP Proportional gain
   * @param kI Integral gain
   * @param kD Differential gain
   */
  @Override
  public void configurePID(double kP, double kI, double kD) {
    config.Slot0.kP = kP;
    config.Slot0.kI = kI;
    config.Slot0.kD = kD;
    PhoenixUtil.tryUntilOk(5, () -> leader.getConfigurator().apply(config, 0.25));
  }

  /**
   * Set the FeedForward portion of the Slot0 closed-loop configuration
   *
   * @param kS Static gain
   * @param kV Velocity gain
   */
  @Override
  public void configureFF(double kS, double kV) {
    config.Slot0.kS = kS;
    config.Slot0.kV = kV;
    PhoenixUtil.tryUntilOk(5, () -> leader.getConfigurator().apply(config, 0.25));
  }

  /**
   * Set the FeedForward portion of the Slot0 closed-loop configuration
   *
   * @param kS Static gain
   * @param kV Velocity gain
   * @param kA Acceleration gain
   */
  public void configureFF(double kS, double kV, double kA) {
    config.Slot0.kS = kS;
    config.Slot0.kV = kV;
    config.Slot0.kA = kA;
    PhoenixUtil.tryUntilOk(5, () -> leader.getConfigurator().apply(config, 0.25));
  }
}
