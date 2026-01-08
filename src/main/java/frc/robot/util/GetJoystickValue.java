// Copyright (c) 2024-2026 Az-FIRST
// http://github.com/AZ-First
//
// Use of this source code is governed by a BSD
// license that can be found in the AdvantageKit-License.md file
// at the root directory of this project.

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
