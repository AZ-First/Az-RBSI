// Copyright (c) 2024 Az-FIRST
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

package frc.robot.subsystems.drive;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.util.YagslConstants;

/**
 * Holds the proper set of drive constants given the type of drive
 *
 * <p>Does some translation of how the two keep values to return a completely unified API. This file
 * should not be modified.
 */
public class DriveConstants {

  // Declare all the constants
  public static final double kCoupleRatio;
  public static final double kDriveGearRatio;
  public static final double kSteerGearRatio;
  public static final double kWheelRadiusInches;
  public static final String kCANbusName;
  public static final int kPigeonId;
  public static final double kSteerInertia;
  public static final double kDriveInertia;
  public static final double kSteerFrictionVoltage;
  public static final double kDriveFrictionVoltage;
  public static final double kSteerCurrentLimit;
  public static final double kDriveCurrentLimit;
  public static final double kOptimalVoltage;
  public static final int kFrontLeftDriveMotorId;
  public static final int kFrontLeftSteerMotorId;
  public static final int kFrontLeftEncoderId;
  public static final String kFrontLeftDriveCanbus;
  public static final String kFrontLeftSteerCanbus;
  public static final String kFrontLeftEncoderCanbus;
  public static final String kFrontLeftDriveType;
  public static final String kFrontLeftSteerType;
  public static final String kFrontLeftEncoderType;
  public static final double kFrontLeftEncoderOffset; // In Radians
  public static final boolean kFrontLeftDriveInvert;
  public static final boolean kFrontLeftSteerInvert;
  public static final boolean kFrontLeftEncoderInvert;
  public static final double kFrontLeftXPosInches;
  public static final double kFrontLeftYPosInches;
  public static final int kFrontRightDriveMotorId;
  public static final int kFrontRightSteerMotorId;
  public static final int kFrontRightEncoderId;
  public static final String kFrontRightDriveCanbus;
  public static final String kFrontRightSteerCanbus;
  public static final String kFrontRightEncoderCanbus;
  public static final String kFrontRightDriveType;
  public static final String kFrontRightSteerType;
  public static final String kFrontRightEncoderType;
  public static final double kFrontRightEncoderOffset; // In Radians
  public static final boolean kFrontRightDriveInvert;
  public static final boolean kFrontRightSteerInvert;
  public static final boolean kFrontRightEncoderInvert;
  public static final double kFrontRightXPosInches;
  public static final double kFrontRightYPosInches;
  public static final int kBackLeftDriveMotorId;
  public static final int kBackLeftSteerMotorId;
  public static final int kBackLeftEncoderId;
  public static final String kBackLeftDriveCanbus;
  public static final String kBackLeftSteerCanbus;
  public static final String kBackLeftEncoderCanbus;
  public static final String kBackLeftDriveType;
  public static final String kBackLeftSteerType;
  public static final String kBackLeftEncoderType;
  public static final double kBackLeftEncoderOffset; // In Radians
  public static final boolean kBackLeftDriveInvert;
  public static final boolean kBackLeftSteerInvert;
  public static final boolean kBackLeftEncoderInvert;
  public static final double kBackLeftXPosInches;
  public static final double kBackLeftYPosInches;
  public static final int kBackRightDriveMotorId;
  public static final int kBackRightSteerMotorId;
  public static final int kBackRightEncoderId;
  public static final String kBackRightDriveCanbus;
  public static final String kBackRightSteerCanbus;
  public static final String kBackRightEncoderCanbus;
  public static final String kBackRightDriveType;
  public static final String kBackRightSteerType;
  public static final String kBackRightEncoderType;
  public static final double kBackRightEncoderOffset; // In Radians
  public static final boolean kBackRightDriveInvert;
  public static final boolean kBackRightSteerInvert;
  public static final boolean kBackRightEncoderInvert;
  public static final double kBackRightXPosInches;
  public static final double kBackRightYPosInches;
  public static final double kDriveP;
  public static final double kDriveI;
  public static final double kDriveD;
  public static final double kDriveF;
  public static final double kDriveIZ;
  public static final double kSteerP;
  public static final double kSteerI;
  public static final double kSteerD;
  public static final double kSteerF;
  public static final double kSteerIZ;

  // Fill in the values from the proper source
  static {
    switch (Constants.getSwerveType()) {
      case PHOENIX6:
        kCoupleRatio = TunerConstants.kCoupleRatio;
        kDriveGearRatio = TunerConstants.kDriveGearRatio;
        kSteerGearRatio = TunerConstants.kSteerGearRatio;
        kWheelRadiusInches = TunerConstants.kWheelRadiusInches;
        kCANbusName = TunerConstants.kCANbusName;
        kPigeonId = TunerConstants.kPigeonId;
        kSteerInertia = TunerConstants.kSteerInertia;
        kDriveInertia = TunerConstants.kDriveInertia;
        kSteerFrictionVoltage = TunerConstants.kSteerFrictionVoltage;
        kDriveFrictionVoltage = TunerConstants.kDriveFrictionVoltage;
        kSteerCurrentLimit = 40.0; // Example from CTRE documentation
        kDriveCurrentLimit = 120.0; // Example from CTRE documentation
        kOptimalVoltage = 12.0; // Assumed Ideal
        kFrontLeftDriveMotorId = TunerConstants.kFrontLeftDriveMotorId;
        kFrontLeftSteerMotorId = TunerConstants.kFrontLeftSteerMotorId;
        kFrontLeftEncoderId = TunerConstants.kFrontLeftEncoderId;
        kFrontLeftDriveCanbus = TunerConstants.kCANbusName;
        kFrontLeftSteerCanbus = TunerConstants.kCANbusName;
        kFrontLeftEncoderCanbus = TunerConstants.kCANbusName;
        kFrontLeftDriveType = "kraken";
        kFrontLeftSteerType = "kraken";
        kFrontLeftEncoderType = "cancoder";
        kFrontLeftEncoderOffset = Units.rotationsToRadians(TunerConstants.kFrontLeftEncoderOffset);
        kFrontLeftDriveInvert = TunerConstants.kInvertLeftSide;
        kFrontLeftSteerInvert = TunerConstants.kFrontLeftSteerInvert;
        kFrontLeftEncoderInvert = false;
        kFrontLeftXPosInches = TunerConstants.kFrontLeftXPosInches;
        kFrontLeftYPosInches = TunerConstants.kFrontLeftYPosInches;
        kFrontRightDriveMotorId = TunerConstants.kFrontRightDriveMotorId;
        kFrontRightSteerMotorId = TunerConstants.kFrontRightSteerMotorId;
        kFrontRightEncoderId = TunerConstants.kFrontRightEncoderId;
        kFrontRightDriveCanbus = TunerConstants.kCANbusName;
        kFrontRightSteerCanbus = TunerConstants.kCANbusName;
        kFrontRightEncoderCanbus = TunerConstants.kCANbusName;
        kFrontRightDriveType = "kraken";
        kFrontRightSteerType = "kraken";
        kFrontRightEncoderType = "cancoder";
        kFrontRightEncoderOffset =
            Units.rotationsToRadians(TunerConstants.kFrontRightEncoderOffset);
        kFrontRightDriveInvert = TunerConstants.kInvertRightSide;
        kFrontRightSteerInvert = TunerConstants.kFrontRightSteerInvert;
        kFrontRightEncoderInvert = false;
        kFrontRightXPosInches = TunerConstants.kFrontRightXPosInches;
        kFrontRightYPosInches = TunerConstants.kFrontRightYPosInches;
        kBackLeftDriveMotorId = TunerConstants.kBackLeftDriveMotorId;
        kBackLeftSteerMotorId = TunerConstants.kBackLeftSteerMotorId;
        kBackLeftEncoderId = TunerConstants.kBackLeftEncoderId;
        kBackLeftDriveCanbus = TunerConstants.kCANbusName;
        kBackLeftSteerCanbus = TunerConstants.kCANbusName;
        kBackLeftEncoderCanbus = TunerConstants.kCANbusName;
        kBackLeftDriveType = "kraken";
        kBackLeftSteerType = "kraken";
        kBackLeftEncoderType = "cancoder";
        kBackLeftEncoderOffset = Units.rotationsToRadians(TunerConstants.kBackLeftEncoderOffset);
        kBackLeftDriveInvert = TunerConstants.kInvertLeftSide;
        kBackLeftSteerInvert = TunerConstants.kBackLeftSteerInvert;
        kBackLeftEncoderInvert = false;
        kBackLeftXPosInches = TunerConstants.kBackLeftXPosInches;
        kBackLeftYPosInches = TunerConstants.kBackLeftYPosInches;
        kBackRightDriveMotorId = TunerConstants.kBackRightDriveMotorId;
        kBackRightSteerMotorId = TunerConstants.kBackRightSteerMotorId;
        kBackRightEncoderId = TunerConstants.kBackRightEncoderId;
        kBackRightDriveCanbus = TunerConstants.kCANbusName;
        kBackRightSteerCanbus = TunerConstants.kCANbusName;
        kBackRightEncoderCanbus = TunerConstants.kCANbusName;
        kBackRightDriveType = "kraken";
        kBackRightSteerType = "kraken";
        kBackRightEncoderType = "cancoder";
        kBackRightEncoderOffset = Units.rotationsToRadians(TunerConstants.kBackRightEncoderOffset);
        kBackRightDriveInvert = TunerConstants.kInvertRightSide;
        kBackRightSteerInvert = TunerConstants.kBackRightSteerInvert;
        kBackRightEncoderInvert = false;
        kBackRightXPosInches = TunerConstants.kBackRightXPosInches;
        kBackRightYPosInches = TunerConstants.kBackRightYPosInches;
        kDriveP = TunerConstants.driveGains.kP;
        kDriveI = TunerConstants.driveGains.kI;
        kDriveD = TunerConstants.driveGains.kD;
        kDriveF = TunerConstants.driveGains.kV;
        kDriveIZ = 0.0;
        kSteerP = TunerConstants.steerGains.kP;
        kSteerI = TunerConstants.steerGains.kI;
        kSteerD = TunerConstants.steerGains.kD;
        kSteerF = TunerConstants.steerGains.kV;
        kSteerIZ = 0.0;
        break;

      case YAGSL:
        kCoupleRatio = YagslConstants.kCoupleRatio;
        kDriveGearRatio = YagslConstants.kDriveGearRatio;
        kSteerGearRatio = YagslConstants.kSteerGearRatio;
        kWheelRadiusInches = YagslConstants.kWheelRadiusInches;
        kCANbusName = YagslConstants.kCANbusName;
        kPigeonId = YagslConstants.kPigeonId;
        kSteerInertia = YagslConstants.kSteerInertia;
        kDriveInertia = YagslConstants.kDriveInertia;
        kSteerFrictionVoltage = YagslConstants.kSteerFrictionVoltage;
        kDriveFrictionVoltage = YagslConstants.kDriveFrictionVoltage;
        kSteerCurrentLimit = YagslConstants.kSteerCurrentLimit;
        kDriveCurrentLimit = YagslConstants.kDriveCurrentLimit;
        kOptimalVoltage = YagslConstants.kOptimalVoltage;
        kFrontLeftDriveMotorId = YagslConstants.kFrontLeftDriveMotorId;
        kFrontLeftSteerMotorId = YagslConstants.kFrontLeftSteerMotorId;
        kFrontLeftEncoderId = YagslConstants.kFrontLeftEncoderId;
        kFrontLeftDriveCanbus = YagslConstants.kFrontLeftDriveCanbus;
        kFrontLeftSteerCanbus = YagslConstants.kFrontLeftSteerCanbus;
        kFrontLeftEncoderCanbus = YagslConstants.kFrontLeftEncoderCanbus;
        kFrontLeftDriveType = YagslConstants.kFrontLeftDriveType.toLowerCase();
        kFrontLeftSteerType = YagslConstants.kFrontLeftSteerType.toLowerCase();
        kFrontLeftEncoderType = YagslConstants.kFrontLeftEncoderType.toLowerCase();
        kFrontLeftEncoderOffset = Units.degreesToRadians(YagslConstants.kFrontLeftEncoderOffset);
        kFrontLeftDriveInvert = YagslConstants.kFrontLeftDriveInvert;
        kFrontLeftSteerInvert = YagslConstants.kFrontLeftSteerInvert;
        kFrontLeftEncoderInvert = YagslConstants.kFrontLeftEncoderInvert;
        kFrontLeftXPosInches = YagslConstants.kFrontLeftXPosInches;
        kFrontLeftYPosInches = YagslConstants.kFrontLeftYPosInches;
        kFrontRightDriveMotorId = YagslConstants.kFrontRightDriveMotorId;
        kFrontRightSteerMotorId = YagslConstants.kFrontRightSteerMotorId;
        kFrontRightEncoderId = YagslConstants.kFrontRightEncoderId;
        kFrontRightDriveCanbus = YagslConstants.kFrontRightDriveCanbus;
        kFrontRightSteerCanbus = YagslConstants.kFrontRightSteerCanbus;
        kFrontRightEncoderCanbus = YagslConstants.kFrontRightEncoderCanbus;
        kFrontRightDriveType = YagslConstants.kFrontRightDriveType.toLowerCase();
        kFrontRightSteerType = YagslConstants.kFrontRightSteerType.toLowerCase();
        kFrontRightEncoderType = YagslConstants.kFrontRightEncoderType.toLowerCase();
        kFrontRightEncoderOffset = Units.degreesToRadians(YagslConstants.kFrontRightEncoderOffset);
        kFrontRightDriveInvert = YagslConstants.kFrontRightDriveInvert;
        kFrontRightSteerInvert = YagslConstants.kFrontRightSteerInvert;
        kFrontRightEncoderInvert = YagslConstants.kFrontRightEncoderInvert;
        kFrontRightXPosInches = YagslConstants.kFrontRightXPosInches;
        kFrontRightYPosInches = YagslConstants.kFrontRightYPosInches;
        kBackLeftDriveMotorId = YagslConstants.kBackLeftDriveMotorId;
        kBackLeftSteerMotorId = YagslConstants.kBackLeftSteerMotorId;
        kBackLeftEncoderId = YagslConstants.kBackLeftEncoderId;
        kBackLeftDriveCanbus = YagslConstants.kBackLeftDriveCanbus;
        kBackLeftSteerCanbus = YagslConstants.kBackLeftSteerCanbus;
        kBackLeftEncoderCanbus = YagslConstants.kBackLeftEncoderCanbus;
        kBackLeftDriveType = YagslConstants.kBackLeftDriveType.toLowerCase();
        kBackLeftSteerType = YagslConstants.kBackLeftSteerType.toLowerCase();
        kBackLeftEncoderType = YagslConstants.kBackLeftEncoderType.toLowerCase();
        kBackLeftEncoderOffset = Units.degreesToRadians(YagslConstants.kBackLeftEncoderOffset);
        kBackLeftDriveInvert = YagslConstants.kBackLeftDriveInvert;
        kBackLeftSteerInvert = YagslConstants.kBackLeftSteerInvert;
        kBackLeftEncoderInvert = YagslConstants.kBackLeftEncoderInvert;
        kBackLeftXPosInches = YagslConstants.kBackLeftXPosInches;
        kBackLeftYPosInches = YagslConstants.kBackLeftYPosInches;
        kBackRightDriveMotorId = YagslConstants.kBackRightDriveMotorId;
        kBackRightSteerMotorId = YagslConstants.kBackRightSteerMotorId;
        kBackRightEncoderId = YagslConstants.kBackRightEncoderId;
        kBackRightDriveCanbus = YagslConstants.kBackRightDriveCanbus;
        kBackRightSteerCanbus = YagslConstants.kBackRightSteerCanbus;
        kBackRightEncoderCanbus = YagslConstants.kBackRightEncoderCanbus;
        kBackRightDriveType = YagslConstants.kBackRightDriveType.toLowerCase();
        kBackRightSteerType = YagslConstants.kBackRightSteerType.toLowerCase();
        kBackRightEncoderType = YagslConstants.kBackRightEncoderType.toLowerCase();
        kBackRightEncoderOffset = Units.degreesToRadians(YagslConstants.kBackRightEncoderOffset);
        kBackRightDriveInvert = YagslConstants.kBackRightDriveInvert;
        kBackRightSteerInvert = YagslConstants.kBackRightSteerInvert;
        kBackRightEncoderInvert = YagslConstants.kBackRightEncoderInvert;
        kBackRightXPosInches = YagslConstants.kBackRightXPosInches;
        kBackRightYPosInches = YagslConstants.kBackRightYPosInches;
        kDriveP = YagslConstants.kDriveP;
        kDriveI = YagslConstants.kDriveI;
        kDriveD = YagslConstants.kDriveD;
        kDriveF = YagslConstants.kDriveF;
        kDriveIZ = YagslConstants.kDriveIZ;
        kSteerP = YagslConstants.kSteerP;
        kSteerI = YagslConstants.kSteerI;
        kSteerD = YagslConstants.kSteerD;
        kSteerF = YagslConstants.kSteerF;
        kSteerIZ = YagslConstants.kSteerIZ;
        break;

      default:
        throw new RuntimeException("Invalid Swerve Drive Type");
    }
  }

  // Computed quantities
  public static final double kDriveBaseRadius =
      Math.hypot(kFrontLeftXPosInches, kFrontLeftYPosInches);
}