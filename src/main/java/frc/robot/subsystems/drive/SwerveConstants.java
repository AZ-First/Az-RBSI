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

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Constants.CANBuses;
import frc.robot.generated.TunerFactory;
import frc.robot.subsystems.imu.ImuIO;
import frc.robot.subsystems.imu.ImuIONavX;
import frc.robot.subsystems.imu.ImuIOPigeon2;
import frc.robot.util.YagslConstants;
import java.util.function.Supplier;

/**
 * Holds the proper set of drive constants given the type of drive
 *
 * <p>Does some translation of how the two keep values to return a completely unified API. This file
 * should not be modified.
 */
public class SwerveConstants {

  // Declare all the constants
  public static final String kImuType;
  public static final double kCoupleRatio;
  public static final double kDriveGearRatio;
  public static final double kSteerGearRatio;
  public static final String kCANbusName;
  public static final int kPigeonId;
  public static final double kSteerInertia;
  public static final double kDriveInertia;
  public static final double kSteerFrictionVoltage;
  public static final double kDriveFrictionVoltage;
  public static final double kSteerCurrentLimit;
  public static final double kDriveCurrentLimit;
  public static final int kFLDriveMotorId;
  public static final int kFLSteerMotorId;
  public static final int kFLEncoderId;
  public static final String kFLDriveCanbus;
  public static final String kFLSteerCanbus;
  public static final String kFLEncoderCanbus;
  public static final String kFLDriveType;
  public static final String kFLSteerType;
  public static final String kFLEncoderType;
  public static final double kFLEncoderOffset; // In Radians
  public static final boolean kFLDriveInvert;
  public static final boolean kFLSteerInvert;
  public static final boolean kFLEncoderInvert;
  public static final double kFLXPosMeters;
  public static final double kFLYPosMeters;
  public static final int kFRDriveMotorId;
  public static final int kFRSteerMotorId;
  public static final int kFREncoderId;
  public static final String kFRDriveCanbus;
  public static final String kFRSteerCanbus;
  public static final String kFREncoderCanbus;
  public static final String kFRDriveType;
  public static final String kFRSteerType;
  public static final String kFREncoderType;
  public static final double kFREncoderOffset; // In Radians
  public static final boolean kFRDriveInvert;
  public static final boolean kFRSteerInvert;
  public static final boolean kFREncoderInvert;
  public static final double kFRXPosMeters;
  public static final double kFRYPosMeters;
  public static final int kBLDriveMotorId;
  public static final int kBLSteerMotorId;
  public static final int kBLEncoderId;
  public static final String kBLDriveCanbus;
  public static final String kBLSteerCanbus;
  public static final String kBLEncoderCanbus;
  public static final String kBLDriveType;
  public static final String kBLSteerType;
  public static final String kBLEncoderType;
  public static final double kBLEncoderOffset; // In Radians
  public static final boolean kBLDriveInvert;
  public static final boolean kBLSteerInvert;
  public static final boolean kBLEncoderInvert;
  public static final double kBLXPosMeters;
  public static final double kBLYPosMeters;
  public static final int kBRDriveMotorId;
  public static final int kBRSteerMotorId;
  public static final int kBREncoderId;
  public static final String kBRDriveCanbus;
  public static final String kBRSteerCanbus;
  public static final String kBREncoderCanbus;
  public static final String kBRDriveType;
  public static final String kBRSteerType;
  public static final String kBREncoderType;
  public static final double kBREncoderOffset; // In Radians
  public static final boolean kBRDriveInvert;
  public static final boolean kBRSteerInvert;
  public static final boolean kBREncoderInvert;
  public static final double kBRXPosMeters;
  public static final double kBRYPosMeters;

  // Fill in the values from the proper source
  static {
    var tuner = TunerFactory.INSTANCE;

    switch (Constants.getSwerveType()) {
      case PHOENIX6:
        kImuType = "pigeon2";
        kCoupleRatio = tuner.frontLeft().CouplingGearRatio;
        kDriveGearRatio = tuner.frontLeft().DriveMotorGearRatio;
        kSteerGearRatio = tuner.frontLeft().SteerMotorGearRatio;
        kCANbusName = CANBuses.DRIVE;
        kPigeonId = tuner.drivetrain().Pigeon2Id;
        kSteerInertia = tuner.frontLeft().SteerInertia;
        kDriveInertia = tuner.frontLeft().DriveInertia;
        kSteerFrictionVoltage = 0.0;
        kDriveFrictionVoltage = 0.1;
        kSteerCurrentLimit = 40.0; // Example from CTRE documentation
        kDriveCurrentLimit = 120.0; // Example from CTRE documentation
        // Front Left
        kFLDriveMotorId = tuner.frontLeft().DriveMotorId;
        kFLSteerMotorId = tuner.frontLeft().SteerMotorId;
        kFLEncoderId = tuner.frontLeft().EncoderId;
        kFLDriveCanbus = kCANbusName;
        kFLSteerCanbus = kCANbusName;
        kFLEncoderCanbus = kCANbusName;
        kFLDriveType = "kraken";
        kFLSteerType = "kraken";
        kFLEncoderType = "cancoder";
        kFLEncoderOffset = -Units.rotationsToRadians(tuner.frontLeft().EncoderOffset) + Math.PI;
        kFLDriveInvert = tuner.frontLeft().DriveMotorInverted;
        kFLSteerInvert = tuner.frontLeft().SteerMotorInverted;
        kFLEncoderInvert = tuner.frontLeft().EncoderInverted;
        kFLXPosMeters = tuner.frontLeft().LocationX;
        kFLYPosMeters = tuner.frontLeft().LocationY;
        // Front Right
        kFRDriveMotorId = tuner.frontRight().DriveMotorId;
        kFRSteerMotorId = tuner.frontRight().SteerMotorId;
        kFREncoderId = tuner.frontRight().EncoderId;
        kFRDriveCanbus = kCANbusName;
        kFRSteerCanbus = kCANbusName;
        kFREncoderCanbus = kCANbusName;
        kFRDriveType = "kraken";
        kFRSteerType = "kraken";
        kFREncoderType = "cancoder";
        kFREncoderOffset = -Units.rotationsToRadians(tuner.frontRight().EncoderOffset);
        kFRDriveInvert = tuner.frontRight().DriveMotorInverted;
        kFRSteerInvert = tuner.frontRight().SteerMotorInverted;
        kFREncoderInvert = tuner.frontRight().EncoderInverted;
        kFRXPosMeters = tuner.frontRight().LocationX;
        kFRYPosMeters = tuner.frontRight().LocationY;
        // Back Left
        kBLDriveMotorId = tuner.backLeft().DriveMotorId;
        kBLSteerMotorId = tuner.backLeft().SteerMotorId;
        kBLEncoderId = tuner.backLeft().EncoderId;
        kBLDriveCanbus = kCANbusName;
        kBLSteerCanbus = kCANbusName;
        kBLEncoderCanbus = kCANbusName;
        kBLDriveType = "kraken";
        kBLSteerType = "kraken";
        kBLEncoderType = "cancoder";
        kBLEncoderOffset = -Units.rotationsToRadians(tuner.backLeft().EncoderOffset) + Math.PI;
        kBLDriveInvert = tuner.backLeft().DriveMotorInverted;
        kBLSteerInvert = tuner.backLeft().SteerMotorInverted;
        kBLEncoderInvert = tuner.backLeft().EncoderInverted;
        kBLXPosMeters = tuner.backLeft().LocationX;
        kBLYPosMeters = tuner.backLeft().LocationY;
        // Back Right
        kBRDriveMotorId = tuner.backRight().DriveMotorId;
        kBRSteerMotorId = tuner.backRight().SteerMotorId;
        kBREncoderId = tuner.backRight().EncoderId;
        kBRDriveCanbus = kCANbusName;
        kBRSteerCanbus = kCANbusName;
        kBREncoderCanbus = kCANbusName;
        kBRDriveType = "kraken";
        kBRSteerType = "kraken";
        kBREncoderType = "cancoder";
        kBREncoderOffset = -Units.rotationsToRadians(tuner.backRight().EncoderOffset);
        kBRDriveInvert = tuner.backRight().DriveMotorInverted;
        kBRSteerInvert = tuner.backRight().SteerMotorInverted;
        kBREncoderInvert = tuner.backRight().EncoderInverted;
        kBRXPosMeters = tuner.backRight().LocationX;
        kBRYPosMeters = tuner.backRight().LocationY;
        break;

      case YAGSL:
        kImuType = YagslConstants.swerveDriveJson.imu.type;
        kCoupleRatio = YagslConstants.kCoupleRatio;
        kDriveGearRatio = YagslConstants.kDriveGearRatio;
        kSteerGearRatio = YagslConstants.kSteerGearRatio;
        kCANbusName = YagslConstants.kCANbusName;
        kPigeonId = YagslConstants.kPigeonId;
        kSteerInertia = YagslConstants.kSteerInertia;
        kDriveInertia = YagslConstants.kDriveInertia;
        kSteerFrictionVoltage = YagslConstants.kSteerFrictionVoltage;
        kDriveFrictionVoltage = YagslConstants.kDriveFrictionVoltage;
        kSteerCurrentLimit = YagslConstants.kSteerCurrentLimit;
        kDriveCurrentLimit = YagslConstants.kDriveCurrentLimit;
        // Front Left
        kFLDriveMotorId = YagslConstants.kFrontLeftDriveMotorId;
        kFLSteerMotorId = YagslConstants.kFrontLeftSteerMotorId;
        kFLEncoderId = YagslConstants.kFrontLeftEncoderId;
        kFLDriveCanbus = YagslConstants.kFrontLeftDriveCanbus;
        kFLSteerCanbus = YagslConstants.kFrontLeftSteerCanbus;
        kFLEncoderCanbus = YagslConstants.kFrontLeftEncoderCanbus;
        kFLDriveType = YagslConstants.kFrontLeftDriveType.toLowerCase();
        kFLSteerType = YagslConstants.kFrontLeftSteerType.toLowerCase();
        kFLEncoderType = YagslConstants.kFrontLeftEncoderType.toLowerCase();
        kFLEncoderOffset = Units.degreesToRadians(YagslConstants.kFrontLeftEncoderOffset);
        kFLDriveInvert = YagslConstants.kFrontLeftDriveInvert;
        kFLSteerInvert = YagslConstants.kFrontLeftSteerInvert;
        kFLEncoderInvert = YagslConstants.kFrontLeftEncoderInvert;
        kFLXPosMeters = Units.inchesToMeters(YagslConstants.kFrontLeftXPosInches);
        kFLYPosMeters = Units.inchesToMeters(YagslConstants.kFrontLeftYPosInches);
        // Front Right
        kFRDriveMotorId = YagslConstants.kFrontRightDriveMotorId;
        kFRSteerMotorId = YagslConstants.kFrontRightSteerMotorId;
        kFREncoderId = YagslConstants.kFrontRightEncoderId;
        kFRDriveCanbus = YagslConstants.kFrontRightDriveCanbus;
        kFRSteerCanbus = YagslConstants.kFrontRightSteerCanbus;
        kFREncoderCanbus = YagslConstants.kFrontRightEncoderCanbus;
        kFRDriveType = YagslConstants.kFrontRightDriveType.toLowerCase();
        kFRSteerType = YagslConstants.kFrontRightSteerType.toLowerCase();
        kFREncoderType = YagslConstants.kFrontRightEncoderType.toLowerCase();
        kFREncoderOffset = Units.degreesToRadians(YagslConstants.kFrontRightEncoderOffset);
        kFRDriveInvert = YagslConstants.kFrontRightDriveInvert;
        kFRSteerInvert = YagslConstants.kFrontRightSteerInvert;
        kFREncoderInvert = YagslConstants.kFrontRightEncoderInvert;
        kFRXPosMeters = Units.inchesToMeters(YagslConstants.kFrontRightXPosInches);
        kFRYPosMeters = Units.inchesToMeters(YagslConstants.kFrontRightYPosInches);
        // Back Left
        kBLDriveMotorId = YagslConstants.kBackLeftDriveMotorId;
        kBLSteerMotorId = YagslConstants.kBackLeftSteerMotorId;
        kBLEncoderId = YagslConstants.kBackLeftEncoderId;
        kBLDriveCanbus = YagslConstants.kBackLeftDriveCanbus;
        kBLSteerCanbus = YagslConstants.kBackLeftSteerCanbus;
        kBLEncoderCanbus = YagslConstants.kBackLeftEncoderCanbus;
        kBLDriveType = YagslConstants.kBackLeftDriveType.toLowerCase();
        kBLSteerType = YagslConstants.kBackLeftSteerType.toLowerCase();
        kBLEncoderType = YagslConstants.kBackLeftEncoderType.toLowerCase();
        kBLEncoderOffset = Units.degreesToRadians(YagslConstants.kBackLeftEncoderOffset);
        kBLDriveInvert = YagslConstants.kBackLeftDriveInvert;
        kBLSteerInvert = YagslConstants.kBackLeftSteerInvert;
        kBLEncoderInvert = YagslConstants.kBackLeftEncoderInvert;
        kBLXPosMeters = Units.inchesToMeters(YagslConstants.kBackLeftXPosInches);
        kBLYPosMeters = Units.inchesToMeters(YagslConstants.kBackLeftYPosInches);
        // Back Right
        kBRDriveMotorId = YagslConstants.kBackRightDriveMotorId;
        kBRSteerMotorId = YagslConstants.kBackRightSteerMotorId;
        kBREncoderId = YagslConstants.kBackRightEncoderId;
        kBRDriveCanbus = YagslConstants.kBackRightDriveCanbus;
        kBRSteerCanbus = YagslConstants.kBackRightSteerCanbus;
        kBREncoderCanbus = YagslConstants.kBackRightEncoderCanbus;
        kBRDriveType = YagslConstants.kBackRightDriveType.toLowerCase();
        kBRSteerType = YagslConstants.kBackRightSteerType.toLowerCase();
        kBREncoderType = YagslConstants.kBackRightEncoderType.toLowerCase();
        kBREncoderOffset = Units.degreesToRadians(YagslConstants.kBackRightEncoderOffset);
        kBRDriveInvert = YagslConstants.kBackRightDriveInvert;
        kBRSteerInvert = YagslConstants.kBackRightSteerInvert;
        kBREncoderInvert = YagslConstants.kBackRightEncoderInvert;
        kBRXPosMeters = Units.inchesToMeters(YagslConstants.kBackRightXPosInches);
        kBRYPosMeters = Units.inchesToMeters(YagslConstants.kBackRightYPosInches);
        break;

      default:
        throw new RuntimeException("Invalid Swerve Drive Type");
    }
  }

  // Derived constant AFTER the static block
  public static final ImuType kImu = ImuType.fromString(kImuType);

  // Computed quantities
  public static final double kDriveBaseRadiusMeters =
      Math.max(
          Math.max(
              Math.hypot(kFLXPosMeters, kFLYPosMeters), Math.hypot(kFRXPosMeters, kFRYPosMeters)),
          Math.max(
              Math.hypot(kBLXPosMeters, kBLYPosMeters), Math.hypot(kBRXPosMeters, kBRYPosMeters)));
  public static final double kDriveBaseRadiusInches = Units.metersToInches(kDriveBaseRadiusMeters);

  // Are we on the CANivore or not?
  public static final double kOdometryFrequency =
      new CANBus(kCANbusName).isNetworkFD() ? 250.0 : 100.0;

  // SPARK Drive encoder configuration
  // Rotor Rotations -> Wheel Radians
  public static final double driveEncoderPositionFactor = 2 * Math.PI / kDriveGearRatio;
  // Rotor RPM -> Wheel Rad/Sec
  public static final double driveEncoderVelocityFactor = (2 * Math.PI) / 60.0 / kDriveGearRatio;
  // Rotations -> Radians
  public static final double turnEncoderPositionFactor = 2 * Math.PI;
  // RPM -> Rad/Sec
  public static final double turnEncoderVelocityFactor = (2 * Math.PI) / 60.0;
  // Drive PID configuration
  public static final double driveKv = 0.1;
  // Turn PID configuration0
  public static final double turnPIDMinInput = 0; // Radians
  public static final double turnPIDMaxInput = 2 * Math.PI; // Radians

  /** IMU Type enum */
  public enum ImuType {
    PIGEON(new String[] {"pigeon", "pigeon2"}, ImuIOPigeon2::new),
    NAVX(new String[] {"navx", "navx_spi"}, ImuIONavX::new);

    private final String[] keys;
    public final Supplier<ImuIO> factory;

    ImuType(String[] keys, Supplier<ImuIO> factory) {
      this.keys = keys;
      this.factory = factory;
    }

    public static ImuType fromString(String s) {
      if (s == null) throw new IllegalArgumentException("IMU type string is null");
      String norm = s.trim().toLowerCase();

      for (ImuType t : values()) {
        for (String k : t.keys) {
          if (norm.equals(k)) return t;
        }
      }
      throw new IllegalArgumentException("Unknown IMU type: '" + s + "'");
    }
  }
}
