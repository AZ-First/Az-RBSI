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

package frc.robot.util;

/**
 * This class contains the various RBSI enumerations for robot type and other multiply supported
 * functions (swerve drive type, vision, etc.).
 */
public class RBSIEnum {

  /** Enumerate the robot types (add additional bots here) */
  public static enum RobotType {
    DEVBOT, // Development / Alpha / Practice Bot
    COMPBOT, // Competition robot
    SIMBOT // Simulated robot
  }

  /** Enumerate the robot operation modes */
  public static enum Mode {
    REAL, // REAL == Running on a real robot
    REPLAY, // REPLAY == Replaying from a log file
    SIM // SIM == Running a physics simulator
  }

  /** Enumerate the supported swerve drive types */
  public static enum SwerveType {
    PHOENIX6, // The all-CTRE Phoenix6 swerve generator
    YAGSL // The generic YAGSL swerve generator
  }

  /** Enumerate the supported autonomous path planning types */
  public static enum AutoType {
    PATHPLANNER, // PathPlanner (https://pathplanner.dev/home.html)
    CHOREO // Choreo (https://sleipnirgroup.github.io/Choreo/)
  }

  /** Enumerate the supported vision types */
  public static enum VisionType {
    PHOTON, // PhotonVision (https://docs.photonvision.org/en/latest/)
    LIMELIGHT, // Limelight (https://docs.limelightvision.io/docs/docs-limelight/)
    NONE // No cameras
  }
}