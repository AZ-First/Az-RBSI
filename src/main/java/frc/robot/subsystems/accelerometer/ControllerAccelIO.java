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

package frc.robot.subsystems.accelerometer;

import org.littletonrobotics.junction.AutoLog;

/** Acceleration readings from the robot controller, in SI units. */
public interface ControllerAccelIO {
  @AutoLog
  class ControllerAccelIOInputs {
    public boolean connected;
    public long timestampNs;
    public double xMetersPerSecondSq;
    public double yMetersPerSecondSq;
    public double zMetersPerSecondSq;
  }

  void updateInputs(ControllerAccelIOInputs inputs);

  static ControllerAccelIO noop() {
    return inputs -> {
      inputs.connected = false;
      inputs.timestampNs = 0L;
      inputs.xMetersPerSecondSq = 0.0;
      inputs.yMetersPerSecondSq = 0.0;
      inputs.zMetersPerSecondSq = 0.0;
    };
  }
}
