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

import org.littletonrobotics.junction.Logger;

/** Time utility that works for REAL, SIM, and REPLAY */
public final class TimeUtil {
  private TimeUtil() {}

  /** Always seconds, regardless of real/sim/replay timebase quirks. */
  public static double now() {
    return Logger.getTimestamp() * 1.0e-6;
  }
}
