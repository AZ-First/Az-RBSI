// Copyright (c) 2024-2027 Az-FIRST
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
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;
import org.wpilib.framework.RobotBase;

/**
 * This class is designed to include Az-RBSI specific methods on top of the standard WPILib
 * command-based mechanism classes. Robot mechanisms (e.g., drivebases, flywheels, arms, elevators)
 * should subclass {@code RBSIMechanism} rather than {@code Mechanism} to gain access to added
 * functionality. V3 {@code Mechanism} has no automatic periodic hook: the constructor explicitly
 * registers this class's input and logging callback with the scheduler.
 */
public abstract class RBSIMechanism implements Mechanism {
  private static final int TIMING_LOG_PERIOD_LOOPS = 5;
  private static final int[] NO_POWER_PORTS = {};

  private final String name = getClass().getSimpleName();
  private int timingLogLoops = 0;

  protected RBSIMechanism() {
    Scheduler.getDefault()
        .addPeriodic(() -> {
          periodic();
          if (RobotBase.isSimulation()) {
            simulationPeriodic();
          }
        });
  }

  /**
   * Guaranteed timing wrapper for the explicitly registered callback (cannot be bypassed by
   * subclasses).
   *
   * <p>DO NOT OVERRIDE.
   *
   * <p>Mechanisms must implement {@link #rbsiPeriodic()} instead.
   *
   * <p>If you see a compiler error here, remove your periodic() override and move your logic into
   * rbsiPeriodic().
   */
  public final void periodic() {
    long start = System.nanoTime();
    rbsiPeriodic();
    if (++timingLogLoops >= TIMING_LOG_PERIOD_LOOPS) {
      timingLogLoops = 0;
      Logger.recordOutput(
          "LogPeriodic/Subsystem/" + name + "MS", (System.nanoTime() - start) / 1e6);
    }
  }

  /** Subclasses must implement this instead of periodic(). */
  protected abstract void rbsiPeriodic();

  /** Override to update simulation state after the regular periodic callback. */
  public void simulationPeriodic() {}

  /**
   * Gets the power ports associated with this mechanism.
   *
   * @return Array of power distribution module ports
   */
  public int[] getPowerPorts() {
    return NO_POWER_PORTS;
  }
}
