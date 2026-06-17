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
  private final ConduitApi conduit = ConduitApi.getInstance();

  // Define local variables
  private final LoggedTunableNumber batteryCapacityAh;
  private double totalAmpHours = 0.0;
  private double totalEnergyJoules = 0.0;
  private long lastTimestampUs = RobotController.getFPGATime(); // In microseconds
  private double lastVoltage = 0.0;
  private double lastTotalCurrent = 0.0;
  private boolean totalCurrentOverLimit = false;
  private boolean brownoutImminent = false;

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

  @Override
  protected int getPeriodPriority() {
    return 30;
  }

  /** Periodic Method */
  @Override
  public void rbsiPeriodic() {
    // Limit polling to every 5 loops
    if ((loops++ % 5) != 0) return; // 50Hz loop -> run at 10Hz

    // --- Read voltage & total current ---
    double voltage = conduit.getPDPVoltage();
    double totalCurrent = conduit.getPDPTotalCurrent();
    lastVoltage = voltage;
    lastTotalCurrent = totalCurrent;
    totalCurrentOverLimit = totalCurrent > PowerConstants.kTotalMaxCurrentAmps;

    // --- Safety alerts ---
    totalCurrentAlert.set(totalCurrentOverLimit);
    lowVoltageAlert.set(voltage < PowerConstants.kWarningVoltage);
    criticalVoltageAlert.set(voltage < PowerConstants.kCriticalVoltage);
    Logger.recordOutput("Power/Voltage", voltage);
    Logger.recordOutput("Power/TotalCurrent", totalCurrent);
    Logger.recordOutput("Power/TotalCurrentOverLimit", totalCurrentOverLimit);

    for (int ch = 0; ch < Math.min(conduit.getPDPChannelCount(), portAlerts.length); ch++) {
      portAlerts[ch].set(
          conduit.getPDPChannelCurrent(ch) > PowerConstants.kMotorPortMaxCurrentAmps);
    }

    // --- Battery estimation ---
    long nowUs = RobotController.getFPGATime();
    double dtSec = (nowUs - lastTimestampUs) / 1e6;
    lastTimestampUs = nowUs;

    totalAmpHours += totalCurrent * dtSec / 3600.0; // accumulate amp-hours
    double capacityAh = batteryCapacityAh.getAsDouble();
    double batteryPercent =
        capacityAh > 0.0 ? 100.0 * (capacityAh - totalAmpHours) / capacityAh : 0.0;

    Logger.recordOutput("Power/BatteryPercentEstimate", Math.max(0.0, batteryPercent));
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
    totalEnergyJoules += totalPower * dtSec;
    Logger.recordOutput("Power/TotalPower", totalPower);
    Logger.recordOutput("Power/EnergyJoules", totalEnergyJoules);
    Logger.recordOutput("Power/EnergyWh", totalEnergyJoules / 3600.0);

    // --- Brownout prediction ---
    brownoutImminent = voltage < PowerConstants.kLimitingVoltage;
    Logger.recordOutput("Power/BrownoutImminent", brownoutImminent);
  }

  private void logGroupCurrent(String name, int[] ports) {
    double sum = 0.0;
    for (int port : ports) {
      if (port >= 0 && port < conduit.getPDPChannelCount()) {
        sum += conduit.getPDPChannelCurrent(port);
      }
    }
    Logger.recordOutput("Power/Subsystems/" + name + "_Current", sum);
  }

  /** Returns the most recently sampled battery voltage. */
  public double getLastVoltage() {
    return lastVoltage;
  }

  /** Returns the most recently sampled total current draw. */
  public double getLastTotalCurrent() {
    return lastTotalCurrent;
  }

  /** Returns whether the last sampled total current exceeded the configured warning limit. */
  public boolean isTotalCurrentOverLimit() {
    return totalCurrentOverLimit;
  }

  /**
   * Returns whether the last sampled voltage is below the limiting threshold.
   *
   * <p>Mechanism-specific commands can use this signal to shed load intentionally. The generic
   * power monitor should not stop motors by itself because mechanism priority is game- and
   * robot-specific.
   */
  public boolean isBrownoutImminent() {
    return brownoutImminent;
  }
}
