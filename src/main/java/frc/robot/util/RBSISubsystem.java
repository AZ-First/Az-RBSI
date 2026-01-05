// Copyright (c) 2024-2026 Az-FIRST
// http://github.com/AZ-First
//
// Use of this source code is governed by a BSD
// license that can be found in the AdvantageKit-License.md file
// at the root directory of this project.

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
