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
import java.util.Comparator;
import java.util.List;
import org.littletonrobotics.junction.Logger;

/**
 * Base class for virtual subsystems -- not robot hardware -- that should be treated as subsystems
 */
public abstract class VirtualSubsystem {
  private static final List<VirtualSubsystem> subsystems = new ArrayList<>();
  private static boolean needsSort = false;

  private final String name = getClass().getSimpleName();

  // Load all defined virtual subsystems into a list
  public VirtualSubsystem() {
    subsystems.add(this);
    needsSort = true; // a new subsystem changed ordering
  }

  /**
   * Override to control ordering. Lower runs earlier.
   *
   * <p>Example: IMU inputs -30, Drive odometry -20, Vision -10, Coordinator 0.
   */
  protected int getPeriodPriority() {
    return 0;
  }

  /**
   * Run the periodic functions of each subsystem in the order determined by the getPeriodPriority()
   * of each.
   */
  public static void periodicAll() {
    // Sort once (and again only if new subsystems are constructed)
    if (needsSort) {
      subsystems.sort(
          Comparator.comparingInt(VirtualSubsystem::getPeriodPriority)
              // deterministic tie-break to avoid “random” order when priorities match
              .thenComparingInt(System::identityHashCode));
      needsSort = false;
    }

    // Call each virtual subsystem during robotPeriodic()
    for (VirtualSubsystem subsystem : subsystems) {
      subsystem.periodic();
    }
  }

  /**
   * Guaranteed timing wrapper (cannot be bypassed by subclasses).
   *
   * <p>DO NOT OVERRIDE.
   *
   * <p>Subsystems must implement {@link #rbsiPeriodic()} instead.
   *
   * <p>If you see a compiler error here, remove your periodic() override and move your logic into
   * rbsiPeriodic().
   */
  @Deprecated(forRemoval = false)
  public final void periodic() {
    long start = System.nanoTime();
    rbsiPeriodic();
    // Log the timing for this subsystem
    Logger.recordOutput("Loop/Virtual/" + name + "_ms", (System.nanoTime() - start) / 1e6);
  }

  /** Subclasses must implement this instead of periodic(). */
  protected abstract void rbsiPeriodic();
}
