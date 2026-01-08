// Copyright (c) 2024-2026 Az-FIRST
// http://github.com/AZ-First
// Copyright (c) 2024 FRC 254
// https://github.com/team254
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

import com.ctre.phoenix6.CANBus;

/**
 * Class for wrapping Robot / CAN devices with a name and functionality. Included here are both the
 * CAN ID for devices and the port on the Power Distribution Module for power monitoring and
 * management.
 */
public class RobotDeviceId {
  private final int m_CANDeviceNumber;
  private final String m_CANBus;
  private final Integer m_PowerPort;

  public RobotDeviceId(int CANdeviceNumber, String CANbus, Integer powerPort) {
    m_CANDeviceNumber = CANdeviceNumber;
    m_CANBus = CANbus;
    m_PowerPort = powerPort;
  }

  /** Use the default bus name (empty string) */
  public RobotDeviceId(int CANdeviceNumber, Integer powerPort) {
    this(CANdeviceNumber, "", powerPort);
  }

  /** Get the CAN ID value for a named device */
  public int getDeviceNumber() {
    return m_CANDeviceNumber;
  }

  /** Get the CAN bus name for a named device */
  public String getBus() {
    return m_CANBus;
  }

  /** Get the CTRE CANBus object for a named device */
  public CANBus getCANBus() {
    return new CANBus(m_CANBus);
  }

  /** Get the Power Port for a named device */
  public int getPowerPort() {
    return m_PowerPort;
  }

  /** Check whether two named devices are, in fact, the same */
  public boolean equals(RobotDeviceId other) {
    return other.m_CANDeviceNumber == m_CANDeviceNumber && other.m_CANBus == m_CANBus;
  }
}
