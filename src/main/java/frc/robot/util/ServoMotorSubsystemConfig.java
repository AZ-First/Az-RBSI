// Copyright (c) 2025 Az-FIRST
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

import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class ServoMotorSubsystemConfig {
  public String name = "UNNAMED";
  public RobotDeviceId talonCANID;
  public TalonFXConfiguration fxConfig = new TalonFXConfiguration();

  // Ratio of rotor to units for this talon. rotor * by this ratio should
  // be the units.
  // <1 is reduction
  public double unitToRotorRatio = 1.0;
  public double kMinPositionUnits = Double.NEGATIVE_INFINITY;
  public double kMaxPositionUnits = Double.POSITIVE_INFINITY;

  // Moment of Inertia (KgMetersSquared) for sim
  public double momentOfInertia = 0.5;
}
