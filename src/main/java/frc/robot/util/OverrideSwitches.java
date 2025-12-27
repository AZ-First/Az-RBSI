// Copyright (c) 2024-2026 Az-FIRST
// http://github.com/AZ-First
// Copyright (c) 2024 FRC 6328
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

// NOTE: This module assumes the design described in FRC6328's 2024 Chief
//       Delphi build thread:
//       https://www.chiefdelphi.com/t/frc-6328-mechanical-advantage-2024-build-thread/442736/72
//
// The "Arcade Controller" used by 6328 (https://www.amazon.com/gp/product/B00UUROWWK)
// enumerates as a "Generic USB Controller", mapped in this modules as a GenericHID.
//
// If another type of controller is used, swap in the proper class.

package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Interface for physical override switches on operator console. */
public class OverrideSwitches {
  private final GenericHID consoleSwitches;

  public OverrideSwitches(int port) {
    consoleSwitches = new GenericHID(port);
  }

  /** Returns whether the controller is connected. */
  public boolean isConnected() {
    return consoleSwitches.isConnected()
        && !DriverStation.getJoystickIsXbox(consoleSwitches.getPort())
        && consoleSwitches.getName().equals("Generic   USB  Joystick");
  }

  /** Gets the state of a driver-side switch (0-2 from left to right). */
  public boolean getDriverSwitch(int index) {
    if (index < 0 || index > 2) {
      throw new RuntimeException(
          "Invalid driver override index " + Integer.toString(index) + ". Must be 0-2.");
    }
    return consoleSwitches.getRawButton(index + 1);
  }

  /** Gets the state of an operator-side switch (0-4 from left to right). */
  public boolean getOperatorSwitch(int index) {
    if (index < 0 || index > 4) {
      throw new RuntimeException(
          "Invalid operator override index " + Integer.toString(index) + ". Must be 0-4.");
    }
    return consoleSwitches.getRawButton(index + 8);
  }

  /** Gets the state of the multi-directional switch. */
  public MultiDirectionSwitchState getMultiDirectionSwitch() {
    if (consoleSwitches.getRawButton(4)) {
      return MultiDirectionSwitchState.LEFT;
    } else if (consoleSwitches.getRawButton(5)) {
      return MultiDirectionSwitchState.RIGHT;
    } else {
      return MultiDirectionSwitchState.NEUTRAL;
    }
  }

  /** Returns a trigger for a driver-side switch (0-2 from left to right). */
  public Trigger driverSwitch(int index) {
    return new Trigger(() -> getDriverSwitch(index));
  }

  /** Returns a trigger for an operator-side switch (0-4 from left to right). */
  public Trigger operatorSwitch(int index) {
    return new Trigger(() -> getOperatorSwitch(index));
  }

  /** Returns a trigger for when the multi-directional switch is pushed to the left. */
  public Trigger multiDirectionSwitchLeft() {
    return new Trigger(() -> getMultiDirectionSwitch() == MultiDirectionSwitchState.LEFT);
  }

  /** Returns a trigger for when the multi-directional switch is pushed to the right. */
  public Trigger multiDirectionSwitchRight() {
    return new Trigger(() -> getMultiDirectionSwitch() == MultiDirectionSwitchState.RIGHT);
  }

  /** The state of the multi-directional switch. */
  public static enum MultiDirectionSwitchState {
    LEFT,
    NEUTRAL,
    RIGHT
  }
}
