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
  private static final int TIMING_LOG_PERIOD_LOOPS = 5;
  private static final List<VirtualSubsystem> subsystems = new ArrayList<>();
  private static boolean needsSort = false;
  private static long nextConstructionOrder = 0;

  private final String name = getClass().getSimpleName();
  private final long constructionOrder;
  private int timingLogLoops = 0;

  // Load all defined virtual subsystems into a list
  public VirtualSubsystem() {
    constructionOrder = nextConstructionOrder++;
    subsystems.add(this);
    needsSort = true; // a new subsystem changed ordering
  }

  /**
   * Override to control ordering. Lower runs earlier.
   *
   * <p>Example: IMU inputs -30, Drive odometry -20, Vision -10, telemetry/health monitors +20.
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
              // Preserve construction order when priorities match.
              .thenComparingLong(subsystem -> subsystem.constructionOrder));
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
    if (++timingLogLoops >= TIMING_LOG_PERIOD_LOOPS) {
      timingLogLoops = 0;
      Logger.recordOutput(
          "LogPeriodic/VirtualSubsystem/" + name + "MS", (System.nanoTime() - start) / 1e6);
    }
  }

  /** Subclasses must implement this instead of periodic(). */
  protected abstract void rbsiPeriodic();

  static void resetForTesting() {
    subsystems.clear();
    needsSort = false;
    nextConstructionOrder = 0;
  }
}
