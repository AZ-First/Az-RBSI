// Copyright (c) 2024 Az-FIRST
// http://github.com/AZ-First
// Copyright 2021-2024 FRC 6328
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

package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

/**
 * Module IO implementation for SparkMax drive motor controller, SparkMax turn motor controller (NEO
 * or NEO 550), and CTRE's CANcoder.
 *
 * <p>To calibrate the absolute encoder offsets, point the modules straight (such that forward
 * motion on the drive motor will propel the robot forward) and copy the reported values from the
 * absolute encoders using AdvantageScope. These values are logged under
 * "/Drive/ModuleX/TurnAbsolutePositionRad"
 */
public class ModuleIOSparkCANcoder implements ModuleIO {
  private final CANSparkMax driveSparkMax;
  private final CANSparkMax turnSparkMax;

  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder turnRelativeEncoder;
  private final CANcoder cancoder;
  private final StatusSignal<Double> turnAbsolutePosition;

  private final boolean isTurnMotorInverted;
  private final Rotation2d absoluteEncoderOffset;

  public ModuleIOSparkCANcoder(int index) {
    switch (index) {
      case 0:
        // Front Left
        driveSparkMax = new CANSparkMax(kFrontLeftDriveMotorId, MotorType.kBrushless);
        turnSparkMax = new CANSparkMax(kFrontLeftSteerMotorId, MotorType.kBrushless);
        cancoder = new CANcoder(kFrontLeftEncoderId, kFrontLeftEncoderCanbus);
        absoluteEncoderOffset = new Rotation2d(kFrontLeftEncoderOffset);
        isTurnMotorInverted = kFrontLeftSteerInvert;
        break;

      case 1:
        // Front Right
        driveSparkMax = new CANSparkMax(kFrontRightDriveMotorId, MotorType.kBrushless);
        turnSparkMax = new CANSparkMax(kFrontRightSteerMotorId, MotorType.kBrushless);
        cancoder = new CANcoder(kFrontRightEncoderId, kFrontRightEncoderCanbus);
        absoluteEncoderOffset = new Rotation2d(kFrontRightEncoderOffset);
        isTurnMotorInverted = kFrontRightSteerInvert;
        break;

      case 2:
        // Back Left
        driveSparkMax = new CANSparkMax(kBackLeftDriveMotorId, MotorType.kBrushless);
        turnSparkMax = new CANSparkMax(kBackLeftSteerMotorId, MotorType.kBrushless);
        cancoder = new CANcoder(kBackLeftEncoderId, kBackLeftEncoderCanbus);
        absoluteEncoderOffset = new Rotation2d(kBackLeftEncoderOffset);
        isTurnMotorInverted = kBackLeftSteerInvert;
        break;

      case 3:
        // Back Right
        driveSparkMax = new CANSparkMax(kBackRightDriveMotorId, MotorType.kBrushless);
        turnSparkMax = new CANSparkMax(kBackRightSteerMotorId, MotorType.kBrushless);
        cancoder = new CANcoder(kBackRightEncoderId, kBackRightEncoderCanbus);
        absoluteEncoderOffset = new Rotation2d(kBackRightEncoderOffset);
        isTurnMotorInverted = kBackRightSteerInvert;
        break;

      default:
        throw new RuntimeException("Invalid module index");
    }

    driveSparkMax.restoreFactoryDefaults();
    turnSparkMax.restoreFactoryDefaults();

    driveSparkMax.setCANTimeout(250);
    turnSparkMax.setCANTimeout(250);

    driveEncoder = driveSparkMax.getEncoder();
    turnRelativeEncoder = turnSparkMax.getEncoder();

    turnSparkMax.setInverted(isTurnMotorInverted);
    driveSparkMax.setSmartCurrentLimit((int) kDriveCurrentLimit);
    turnSparkMax.setSmartCurrentLimit((int) kSteerCurrentLimit);
    driveSparkMax.enableVoltageCompensation(kOptimalVoltage);
    turnSparkMax.enableVoltageCompensation(12.0);

    cancoder.getConfigurator().apply(new CANcoderConfiguration());

    driveEncoder.setPosition(0.0);
    driveEncoder.setMeasurementPeriod(10);
    driveEncoder.setAverageDepth(2);

    turnRelativeEncoder.setPosition(0.0);
    turnRelativeEncoder.setMeasurementPeriod(10);
    turnRelativeEncoder.setAverageDepth(2);

    turnAbsolutePosition = cancoder.getAbsolutePosition();
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, turnAbsolutePosition);
    driveSparkMax.setCANTimeout(0);
    turnSparkMax.setCANTimeout(0);

    driveSparkMax.burnFlash();
    turnSparkMax.burnFlash();
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    BaseStatusSignal.refreshAll(turnAbsolutePosition);

    inputs.drivePositionRad =
        Units.rotationsToRadians(driveEncoder.getPosition()) / kDriveGearRatio;
    inputs.driveVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(driveEncoder.getVelocity()) / kDriveGearRatio;
    inputs.driveAppliedVolts = driveSparkMax.getAppliedOutput() * driveSparkMax.getBusVoltage();
    inputs.driveCurrentAmps = new double[] {driveSparkMax.getOutputCurrent()};

    inputs.turnAbsolutePosition =
        Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble())
            .minus(absoluteEncoderOffset);
    inputs.turnPosition =
        Rotation2d.fromRotations(turnRelativeEncoder.getPosition() / kSteerGearRatio);
    inputs.turnVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(turnRelativeEncoder.getVelocity())
            / kSteerGearRatio;
    inputs.turnAppliedVolts = turnSparkMax.getAppliedOutput() * turnSparkMax.getBusVoltage();
    inputs.turnCurrentAmps = new double[] {turnSparkMax.getOutputCurrent()};
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveSparkMax.setVoltage(volts);
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnSparkMax.setVoltage(volts);
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    driveSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    turnSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }
}