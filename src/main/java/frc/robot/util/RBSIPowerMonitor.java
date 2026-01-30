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

import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.PowerConstants;
import frc.robot.Constants.RobotDevices;
import frc.robot.util.Alert.AlertType;
import org.littletonrobotics.conduit.ConduitApi;
import org.littletonrobotics.junction.Logger;

/**
 * Power monitoring virtual subsystem that periodically polls the Power Distribution Module. Each
 * port and the sum total currents are compared with limits defined in the ``Constants.java`` file,
 * and subsystem total currents are also computed based on the power ports listed in
 * ``RobotContainer.java``.
 */
public class RBSIPowerMonitor extends VirtualSubsystem {

  private final RBSISubsystem[] subsystems;
  // private final LoggedPowerDistribution m_pdm =
  //     LoggedPowerDistribution.getInstance(PowerConstants.kPDMCANid, PowerConstants.kPDMType);
  ConduitApi conduit = ConduitApi.getInstance();

  // Define local variables
  private final LoggedTunableNumber batteryCapacityAh;
  private double totalAmpHours = 0.0;
  private long lastTimestampUs = RobotController.getFPGATime(); // In microseconds

  // DRIVE and STEER motor power ports
  private final int[] m_drivePowerPorts = {
    RobotDevices.FL_DRIVE.getPowerPort(),
    RobotDevices.FR_DRIVE.getPowerPort(),
    RobotDevices.BL_DRIVE.getPowerPort(),
    RobotDevices.BR_DRIVE.getPowerPort()
  };
  private final int[] m_steerPowerPorts = {
    RobotDevices.FL_ROTATION.getPowerPort(),
    RobotDevices.FR_ROTATION.getPowerPort(),
    RobotDevices.BL_ROTATION.getPowerPort(),
    RobotDevices.BR_ROTATION.getPowerPort()
  };

  private final Alert totalCurrentAlert =
      new Alert("Total current draw exceeds limit!", AlertType.WARNING);
  private final Alert[] portAlerts = new Alert[24]; // or pdh.getNumChannels() after construct
  private final Alert lowVoltageAlert = new Alert("Low battery voltage!", AlertType.WARNING);
  private final Alert criticalVoltageAlert =
      new Alert("Critical battery voltage!", AlertType.ERROR);

  private long loops = 0;

  // Constructor, including inputs of optional subsystems
  public RBSIPowerMonitor(LoggedTunableNumber batteryCapacityAh, RBSISubsystem... subsystems) {
    this.batteryCapacityAh = batteryCapacityAh;
    this.subsystems = subsystems;

    for (int i = 0; i < portAlerts.length; i++) {
      portAlerts[i] = new Alert("Port " + i + " current exceeds limit!", AlertType.WARNING);
    }
  }

  /** Periodic Method */
  @Override
  public void rbsiPeriodic() {
    // Limit polling to every 5 loops
    if ((loops++ % 5) != 0) return; // 50Hz loop -> run at 10Hz

    // --- Read voltage & total current ---
    double voltage = conduit.getPDPVoltage();
    double totalCurrent = conduit.getPDPTotalCurrent();

    // --- Safety alerts ---
    totalCurrentAlert.set(totalCurrent > PowerConstants.kTotalMaxCurrent);
    lowVoltageAlert.set(voltage < PowerConstants.kVoltageWarning);
    criticalVoltageAlert.set(voltage < PowerConstants.kVoltageCritical);

    for (int ch = 0; ch < conduit.getPDPChannelCount(); ch++) {
      portAlerts[ch].set(conduit.getPDPChannelCurrent(ch) > PowerConstants.kMotorPortMaxCurrent);
    }

    // --- Battery estimation ---
    long nowUs = RobotController.getFPGATime();
    double dtSec = (nowUs - lastTimestampUs) / 1e6;
    lastTimestampUs = nowUs;

    totalAmpHours += totalCurrent * dtSec / 3600.0; // accumulate amp-hours
    double batteryPercent =
        100.0 * (batteryCapacityAh.getAsDouble() - totalAmpHours) / batteryCapacityAh.getAsDouble();

    Logger.recordOutput("Power/BatteryPercentEstimate", batteryPercent);
    Logger.recordOutput("Power/AmpHoursUsed", totalAmpHours);

    // --- Drive & Steer aggregation ---
    logGroupCurrent("Drive", m_drivePowerPorts);
    logGroupCurrent("Steer", m_steerPowerPorts);

    // --- Subsystems ---
    for (RBSISubsystem subsystem : subsystems) {
      logGroupCurrent(subsystem.getName(), subsystem.getPowerPorts());
    }

    // --- Energy / power calculations ---
    double totalPower = voltage * totalCurrent; // Watts
    Logger.recordOutput("Power/TotalPower", totalPower);
    Logger.recordOutput("Power/EnergyJoules", totalPower * dtSec);
    Logger.recordOutput("Power/EnergyWh", totalPower * dtSec / 3600.0);

    // --- Brownout prediction ---
    boolean brownoutImminent = voltage < PowerConstants.kVoltageLimiting;
    Logger.recordOutput("Power/BrownoutImminent", brownoutImminent);

    // --- Optional hooks for current shedding ---
    if (brownoutImminent) {
      // TODO: implement automatic shedding: e.g., disable non-critical subsystems
    }
  }

  private void logGroupCurrent(String name, int[] ports) {
    double sum = 0.0;
    for (int port : ports) {
      sum += conduit.getPDPChannelCurrent(port);
    }
    Logger.recordOutput("Power/Subsystems/" + name + "Current", sum);
  }

  // TODO: Do something about setting priorities if drawing too much current

}
