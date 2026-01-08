// Copyright (c) 2024-2026 Az-FIRST
// http://github.com/AZ-First
// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the AdvantageKit-License.md file
// at the root directory of this project.

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
