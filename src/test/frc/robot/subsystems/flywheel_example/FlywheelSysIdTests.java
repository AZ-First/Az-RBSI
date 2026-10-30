// Copyright (c) 2024-2027 Az-FIRST
// http://github.com/AZ-First
//
// Use of this source code is governed by a BSD
// license that can be found in the AdvantageKit-License.md file
// at the root directory of this project.

package frc.robot.subsystems.flywheel_example;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.sysid.SysIdRoutine;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.simulation.DriverStationSim;

class FlywheelSysIdTests {
  @Test
  void voltageAndDutyCycleRoutinesHaveDistinctNamesAndRequireFlywheel() {
    Flywheel flywheel = new Flywheel(new FlywheelIO() {});

    Command voltageForward = flywheel.sysIdVoltageQuasistatic(SysIdRoutine.Direction.FORWARD);
    Command voltageReverse = flywheel.sysIdVoltageDynamic(SysIdRoutine.Direction.REVERSE);
    Command dutyForward = flywheel.sysIdDutyCycleQuasistatic(SysIdRoutine.Direction.FORWARD);
    Command dutyReverse = flywheel.sysIdDutyCycleDynamic(SysIdRoutine.Direction.REVERSE);

    assertEquals("Flywheel SysId Voltage Forward Quasistatic", voltageForward.name());
    assertEquals("Flywheel SysId Voltage Reverse Dynamic", voltageReverse.name());
    assertEquals("Flywheel SysId Duty Cycle Forward Quasistatic", dutyForward.name());
    assertEquals("Flywheel SysId Duty Cycle Reverse Dynamic", dutyReverse.name());
    assertTrue(voltageForward.requires(flywheel));
    assertTrue(voltageReverse.requires(flywheel));
    assertTrue(dutyForward.requires(flywheel));
    assertTrue(dutyReverse.requires(flywheel));
  }

  @Test
  void dynamicVoltageCommandDrivesThenZerosOutputWhenCanceled() {
    assertTrue(HAL.initialize());
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    Scheduler scheduler = Scheduler.createIndependentScheduler();
    RecordingFlywheelIO io = new RecordingFlywheelIO();
    Flywheel flywheel =
        new Flywheel(io) {
          @Override
          public Scheduler getRegisteredScheduler() {
            return scheduler;
          }
        };
    Command command = flywheel.sysIdVoltageDynamic(SysIdRoutine.Direction.FORWARD);

    try {
      scheduler.schedule(command);
      scheduler.run();
      scheduler.run();
      assertEquals(7.0, io.volts, 1e-9);

      scheduler.cancel(command);
      assertEquals(0.0, io.volts, 1e-9);
    } finally {
      scheduler.cancelAll();
      DriverStationSim.setEnabled(false);
      DriverStationSim.notifyNewData();
    }
  }

  private static class RecordingFlywheelIO implements FlywheelIO {
    double volts;

    @Override
    public void setVoltage(double volts) {
      this.volts = volts;
    }
  }
}
