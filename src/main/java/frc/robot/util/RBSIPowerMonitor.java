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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.CANandPowerPorts;
import frc.robot.Constants.PowerConstants;
import frc.robot.util.Alert.AlertType;
import org.littletonrobotics.conduit.ConduitApi;
import org.littletonrobotics.junction.LoggedPowerDistribution;
import org.littletonrobotics.junction.Logger;

/**
 * Power monitoring virtual subsystem that periodically polls the Power Distribution Module. Each
 * port and the sum total currents are compared with limits defined in the ``Constants.java`` file,
 * and subsystem total currents are also computed based on the power ports listed in
 * ``RobotContainer.java``.
 */
public class RBSIPowerMonitor extends VirtualSubsystem {

  private final RBSISubsystem[] subsystems;

  // Get the AdvantageKit conduit for pulling PDM information
  @SuppressWarnings("unused")
  private LoggedPowerDistribution loggedPowerDistribution = LoggedPowerDistribution.getInstance();

  private ConduitApi conduit = ConduitApi.getInstance();

  // Define local variables
  private int NUM_PDH_CHANNELS = conduit.getPDPChannelCount();
  private double[] channelCurrents = new double[NUM_PDH_CHANNELS];
  private double totalAmpHours = 0.0;
  private long lastTimestamp = RobotController.getFPGATime(); // In microseconds
  private double deltaTime = 0.0; // In seconds
  private double batteryVoltage;
  private double totalCurrent;
  private double driveCurrent;
  private double steerCurrent;
  private double subsystemCurrent;
  private LoggedTunableNumber batteryCapacity;
  private double batteryPercent;

  // DRIVE and STEER motor power ports
  private final int[] m_drivePowerPorts = {
    CANandPowerPorts.FL_DRIVE.getPowerPort(),
    CANandPowerPorts.FR_DRIVE.getPowerPort(),
    CANandPowerPorts.BL_DRIVE.getPowerPort(),
    CANandPowerPorts.BR_DRIVE.getPowerPort()
  };
  private final int[] m_steerPowerPorts = {
    CANandPowerPorts.FL_ROTATION.getPowerPort(),
    CANandPowerPorts.FR_ROTATION.getPowerPort(),
    CANandPowerPorts.BL_ROTATION.getPowerPort(),
    CANandPowerPorts.BR_ROTATION.getPowerPort()
  };

  // Class method definition, including inputs of optional subsystems
  public RBSIPowerMonitor(LoggedTunableNumber batteryCapacity, RBSISubsystem... subsystems) {
    this.batteryCapacity = batteryCapacity;
    this.subsystems = subsystems;
  }

  /** Periodic Method */
  public void periodic() {

    // Check the total robot current and individual port currents against Constants
    batteryVoltage = conduit.getPDPVoltage();
    totalCurrent = conduit.getPDPTotalCurrent();
    if (totalCurrent > PowerConstants.kTotalMaxCurrent) {
      new Alert("Total current draw exceeds limit!", AlertType.WARNING).set(true);
    }
    for (int i = 0; i < NUM_PDH_CHANNELS; i++) {
      channelCurrents[i] = conduit.getPDPChannelCurrent(i);
      if (channelCurrents[i] > PowerConstants.kMotorPortMaxCurrent) {
        new Alert("Port " + i + " current draw exceeds limit!", AlertType.WARNING).set(true);
      }
    }
    // Compute the total energy (charge) used in this loop
    deltaTime =
        (RobotController.getFPGATime() - lastTimestamp) / 1.0e6; // Time in seconds since last loop
    lastTimestamp = RobotController.getFPGATime(); // Update timestamp
    totalAmpHours += (totalCurrent * deltaTime / 3600.);

    // Compute estimated battery life left as a percent
    batteryPercent =
        (batteryCapacity.getAsDouble() - totalAmpHours) / batteryCapacity.getAsDouble() * 100;

    // Compute DRIVE and STEER summed currents
    driveCurrent = 0.0;
    steerCurrent = 0.0;
    for (int port : m_drivePowerPorts) {
      driveCurrent += channelCurrents[port];
    }
    for (int port : m_steerPowerPorts) {
      steerCurrent += channelCurrents[port];
    }
    // Log values to AdvantageKit and to SmartDashboard
    Logger.recordOutput("PowerMonitor/Voltage", batteryVoltage);
    Logger.recordOutput("PowerMonitor/EstimatedBatteryPercent", batteryPercent);
    Logger.recordOutput("PowerMonitor/AmpHoursUsed", totalAmpHours);
    Logger.recordOutput("PowerMonitor/TotalCurrent", totalCurrent);
    Logger.recordOutput("PowerMonitor/DriveCurrent", driveCurrent);
    Logger.recordOutput("PowerMonitor/SteerCurrent", steerCurrent);
    SmartDashboard.putNumber("BatteryVoltage", batteryVoltage);
    SmartDashboard.putNumber("EstimatedBatteryPercent", batteryPercent);
    SmartDashboard.putNumber("AmpHoursUsed", totalAmpHours);
    SmartDashboard.putNumber("TotalCurrent", totalCurrent);
    SmartDashboard.putNumber("DriveCurrent", driveCurrent);
    SmartDashboard.putNumber("SteerCurrent", steerCurrent);

    // Compute and log any passed-in subsystems
    for (RBSISubsystem subsystem : subsystems) {
      subsystemCurrent = 0.0;
      for (int port : subsystem.getPowerPorts()) {
        subsystemCurrent += channelCurrents[port];
      }
      Logger.recordOutput("PowerMonitor/" + subsystem.getName() + "Current", subsystemCurrent);
      SmartDashboard.putNumber(subsystem.getName() + "Current", subsystemCurrent);
    }

    // TODO: Do something about setting priorities if drawing too much current

  }
}
