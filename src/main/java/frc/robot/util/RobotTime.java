// Copyright (c) 2025 Az-FIRST
// http://github.com/AZ-First
// Copyright (c) 2024 FRC 254
// https://github.com/team254
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

import org.littletonrobotics.junction.Logger;

public class RobotTime {
  public static double getTimestampSeconds() {
    long micros = Logger.getTimestamp();
    return (double) micros * 1.0E-6;
  }
}
