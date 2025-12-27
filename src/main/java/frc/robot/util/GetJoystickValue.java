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
 * Interface needed to abstraxct away which joystick is used for driving and which for steering with
 * a swerve base. Teams may specify to use the left joystick for either driving or steering in the
 * `Constants.java` file under OperatorConstants.
 */
@FunctionalInterface
public interface GetJoystickValue {
  double value();
}
