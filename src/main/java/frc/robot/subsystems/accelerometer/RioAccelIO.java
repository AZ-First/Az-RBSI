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

package frc.robot.subsystems.accelerometer;

public interface RioAccelIO {
  final class Inputs {
    public boolean connected;
    public long timestampNs;
    public double xG, yG, zG; // raw in g
  }

  default void updateInputs(Inputs inputs) {}

  default void close() {}

  static RioAccelIO noop() {
    return new RioAccelIO() {
      @Override
      public void updateInputs(Inputs inputs) {
        inputs.connected = false;
        inputs.timestampNs = 0L;
        inputs.xG = 0.0;
        inputs.yG = 0.0;
        inputs.zG = 0.0;
      }
    };
  }
}
