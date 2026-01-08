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

package frc.robot.util;

import static frc.robot.subsystems.drive.SwerveConstants.kFLDriveType;
import static frc.robot.subsystems.drive.SwerveConstants.kFLEncoderType;
import static frc.robot.subsystems.drive.SwerveConstants.kFLSteerType;

/** Container to hold various RBSI-specific parsing routines */
public class RBSIParsing {

  /**
   * Parse the module type given the type information for the FL module
   *
   * @return Byte The module type as bits in a byte.
   */
  public static Byte parseModuleType() {
    // NOTE: This assumes all 4 modules have the same arrangement!
    Byte b_drive; // [x,x,-,-,-,-,-,-]
    Byte b_steer; // [-,-,x,x,-,-,-,-]
    Byte b_encoder; // [-,-,-,-,x,x,-,-]
    switch (kFLDriveType) {
      case "falcon":
      case "kraken":
      case "talonfx":
        // CTRE Drive Motor
        b_drive = 0b00;
        break;
      case "sparkmax":
      case "sparkflex":
        // REV Drive Motor
        b_drive = 0b01;
        break;
      default:
        throw new RuntimeException("Invalid drive motor type");
    }
    switch (kFLSteerType) {
      case "falcon":
      case "kraken":
      case "talonfx":
        // CTRE Drive Motor
        b_steer = 0b00;
        break;
      case "sparkmax":
      case "sparkflex":
        // REV Drive Motor
        b_steer = 0b01;
        break;
      default:
        throw new RuntimeException("Invalid steer motor type");
    }
    switch (kFLEncoderType) {
      case "cancoder":
        // CTRE CANcoder
        b_encoder = 0b00;
        break;
      case "analog":
        // Analog Encoder
        b_encoder = 0b01;
        break;
      default:
        throw new RuntimeException("Invalid swerve encoder type");
    }
    return (byte) (0b00000000 | b_drive << 6 | b_steer << 4 | b_encoder << 2);
  }
}
