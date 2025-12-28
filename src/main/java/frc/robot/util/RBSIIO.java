// Copyright (c) 2026 Az-FIRST
// http://github.com/AZ-First
//
// Use of this source code is governed by a BSD
// license that can be found in the AdvantageKit-License.md file
// at the root directory of this project.

package frc.robot.util;

/**
 * This class is designed to provide the base IO methods needed for various subsystems. The goal is
 * to allow the base IO classes to focus on just what that particlar subsystem needs.
 */
public interface RBSIIO {

  /** Define the power ports for this subsystem */
  public final int[] powerPorts = {};

  /** Stop all the motors */
  public default void stop() {}

  /** Set the neutral mode of the motors to COAST */
  public default void setCoast() {}

  /** Set the neutral mode of the motors to BRAKE */
  public default void setBrake() {}

  /** Run open loop at the specified voltage */
  public default void setVoltage(double volts) {}

  /** Run open loop at the specified duty cycle */
  public default void setPercent(double percent) {}

  /** Return the list of PDH power ports used for this mechanism */
  public default int[] getPowerPorts() {
    return powerPorts;
  }
}
