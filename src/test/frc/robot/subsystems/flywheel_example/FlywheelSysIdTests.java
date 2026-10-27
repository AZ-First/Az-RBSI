package frc.robot.subsystems.flywheel_example;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.wpilib.command2.sysid.SysIdRoutine;
import org.wpilib.math.util.Units;

class FlywheelSysIdTests {

  @Test
  void sysIdRoutinesUseDistinctMechanismNames() {
    Flywheel flywheel = new Flywheel(new RecordingFlywheelIO());

    assertEquals(
        "Flywheel SysId Voltage Forward Quasistatic",
        flywheel.sysIdVoltageQuasistatic(SysIdRoutine.Direction.kForward).getName());
    assertEquals(
        "Flywheel SysId Voltage Reverse Dynamic",
        flywheel.sysIdVoltageDynamic(SysIdRoutine.Direction.kReverse).getName());
    assertEquals(
        "Flywheel SysId Duty Cycle Forward Quasistatic",
        flywheel.sysIdDutyCycleQuasistatic(SysIdRoutine.Direction.kForward).getName());
    assertEquals(
        "Flywheel SysId Duty Cycle Reverse Dynamic",
        flywheel.sysIdDutyCycleDynamic(SysIdRoutine.Direction.kReverse).getName());
  }

  @Test
  void velocityCommandsConvertRpmToMechanismRadiansPerSecond() {
    RecordingFlywheelIO io = new RecordingFlywheelIO();
    Flywheel flywheel = new Flywheel(io);

    flywheel.runVelocity(6000.0);
    assertEquals(Units.rotationsPerMinuteToRadiansPerSecond(6000.0), io.velocityRadPerSec);

    flywheel.runVelocityProfiled(3000.0);
    assertEquals(Units.rotationsPerMinuteToRadiansPerSecond(3000.0), io.profiledVelocityRadPerSec);
  }

  private static class RecordingFlywheelIO implements FlywheelIO {
    double velocityRadPerSec;
    double profiledVelocityRadPerSec;

    @Override
    public void setVelocity(double velocityRadPerSec) {
      this.velocityRadPerSec = velocityRadPerSec;
    }

    @Override
    public void setVelocityProfiled(double velocityRadPerSec) {
      this.profiledVelocityRadPerSec = velocityRadPerSec;
    }
  }
}
