package frc.robot.subsystems.flywheel_example;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.wpilib.math.util.Units;

class FlywheelVelocityTests {
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
