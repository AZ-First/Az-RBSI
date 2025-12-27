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

package frc.robot.util;

import java.util.ArrayList;
import java.util.List;

/**
 * Base class for virtual subsystems -- not robot hardware -- that should be treated as subsystems
 */
public abstract class VirtualSubsystem {
  private static List<VirtualSubsystem> subsystems = new ArrayList<>();

  // Load all defined virtual subsystems into a list
  public VirtualSubsystem() {
    subsystems.add(this);
  }

  public static void periodicAll() {
    // Call each virtual subsystem during robotPeriodic()
    for (VirtualSubsystem subsystem : subsystems) {
      subsystem.periodic();
    }
  }

  // Each virtual subsystem must implement its own periodic() method
  public abstract void periodic();
}
