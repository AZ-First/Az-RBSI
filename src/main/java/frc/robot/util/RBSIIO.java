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

/**
 * This class is designed to provide the base IO methods needed for various subsystems. The goal is
 * to allow the base IO classes to focus on just what that particlar subsystem needs.
 */
public interface RBSIIO {

  /** Implementations may override to provide PDH ports. */
  default int[] powerPorts() {
    return new int[] {};
  }

  /** Return the list of PDH power ports used for this mechanism. */
  default int[] getPowerPorts() {
    return powerPorts();
  }

  /** Stop all the motors */
  default void stop() {}

  /** Set the neutral mode of the motors to COAST */
  default void setCoast() {}

  /** Set the neutral mode of the motors to BRAKE */
  default void setBrake() {}

  /** Run open loop at the specified voltage */
  default void setVoltage(double volts) {}

  /** Run open loop at the specified duty cycle */
  default void setPercent(double percent) {}
}
