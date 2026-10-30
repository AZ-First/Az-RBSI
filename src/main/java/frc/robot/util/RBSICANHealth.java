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

import frc.robot.Constants.CANBuses;
import org.littletonrobotics.junction.Logger;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.system.RobotController;

/**
 * Virtual subsystem to monitor CAN health
 *
 * <p>Instantiate one of these subsystems for each CAN bus on the robot in the RobotContainer.
 */
public class RBSICANHealth extends VirtualSubsystem {
  private long loops = 0;
  private final String busName;
  private final CANPort port;

  /** Constructor */
  public RBSICANHealth(String busName) {
    this.busName = busName;
    port = CANBuses.revPort(busName);
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
    var status = RobotController.getCANStatus(port);
    Logger.recordOutput("CAN/" + busName + "/Utilization", status.percentBusUtilization);
    Logger.recordOutput("CAN/" + busName + "/TxFullCount", status.txFullCount);
    Logger.recordOutput("CAN/" + busName + "/RxErrorCount", status.receiveErrorCount);
    Logger.recordOutput("CAN/" + busName + "/TxErrorCount", status.transmitErrorCount);
    Logger.recordOutput("CAN/" + busName + "/BusOffCount", status.busOffCount);
  }
}
