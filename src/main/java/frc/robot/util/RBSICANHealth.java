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

/**
 * Virtual subsystem to monitor CAN health
 *
 * <p>Instantiate one of these subsystems for each CAN bus on the robot in the RobotContainer.
 */
public class RBSICANHealth extends VirtualSubsystem {
  private long loops = 0;
  private final RBSICANBusRegistry.CANBusLike bus;

  /** Constructur */
  public RBSICANHealth(String busName) {
    bus = RBSICANBusRegistry.getLike(busName);
  }

  @Override
  protected int getPeriodPriority() {
    return 20;
  }

  /** Periodic function */
  @Override
  protected void rbsiPeriodic() {
    // Only log CAN health every 5 loops
    if ((loops++ % 5) != 0) return;

    // Get status and log
    var status = bus.getStatus();
    Logger.recordOutput("CAN/" + bus.getName() + "/Utilization", status.BusUtilization);
    Logger.recordOutput("CAN/" + bus.getName() + "/TxFullCount", status.TxFullCount);
    Logger.recordOutput("CAN/" + bus.getName() + "/RxErrorCount", status.REC);
    Logger.recordOutput("CAN/" + bus.getName() + "/TxErrorCount", status.TEC);
    Logger.recordOutput("CAN/" + bus.getName() + "/BusOffCount", status.BusOffCount);
  }
}
