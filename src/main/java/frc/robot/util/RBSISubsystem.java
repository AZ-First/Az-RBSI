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

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * This class is designed to include Az-RBSI specific methods on top of the standard WPILib
 * command-based subsystem classes. All non-drivebase subsystems (e.g., flywheels, arms, elevators,
 * etc.) should subclass ``RBSISubsystem`` rather than ``SubsystemBase`` in order to gain access to
 * added functionality.
 */
public class RBSISubsystem extends SubsystemBase {

  /**
   * Gets the power ports associated with this Subsystem.
   *
   * @return Array of power distribution module ports
   */
  public int[] getPowerPorts() {
    int[] retval = {};
    return retval;
  }
}
