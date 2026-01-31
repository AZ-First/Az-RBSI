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

import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Notifier;

public class RioAccelIORoboRIO implements RioAccelIO {
  private final BuiltInAccelerometer accel = new BuiltInAccelerometer();

  private volatile long stampNs = 0L;
  private volatile double xG = 0.0, yG = 0.0, zG = 0.0;

  private final Notifier sampler;

  public RioAccelIORoboRIO(double sampleHz) {
    sampler =
        new Notifier(
            () -> {
              xG = accel.getX();
              yG = accel.getY();
              zG = accel.getZ();
              stampNs = System.nanoTime();
            });
    sampler.setName("RioAccelSampler");
    sampler.startPeriodic(1.0 / sampleHz);
  }

  @Override
  public void updateInputs(Inputs inputs) {
    inputs.connected = true;
    inputs.timestampNs = stampNs;
    inputs.xG = xG;
    inputs.yG = yG;
    inputs.zG = zG;
  }

  @Override
  public void close() {
    sampler.stop();
  }
}
