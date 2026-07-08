package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.NavigableMap;
import org.junit.jupiter.api.Test;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.system.RobotController;

class UtilTests {
  private static final double EPSILON = 1e-9;

  @Test
  void timeUtilReturnsSecondsFromAdvantageKitMicrosecondTimestamp() {
    assertTrue(HAL.initialize(500, 0));

    double expectedSeconds = RobotController.getTime() * 1.0e-6;
    double actualSeconds = TimeUtil.now();

    assertEquals(expectedSeconds, actualSeconds, 0.050);
  }

  @Test
  void robotDeviceIdUsesValueEqualityAndHandlesMissingPowerPort() {
    RobotDeviceId first = new RobotDeviceId(7, "rio", null);
    RobotDeviceId second = new RobotDeviceId(7, new String("rio"), null);
    RobotDeviceId differentBus = new RobotDeviceId(7, "DriveTrain", null);

    assertEquals(first, second);
    assertEquals(first.hashCode(), second.hashCode());
    assertFalse(first.equals(differentBus));
    assertFalse(first.hasPowerPort());
    assertThrows(IllegalStateException.class, first::getPowerPort);
  }

  @Test
  void robotDeviceIdLooksUpCanBusThroughRegistry() {
    RBSICANBusRegistry.initSim("rio");
    RobotDeviceId device = new RobotDeviceId(1, "rio", 0);

    assertThrows(IllegalStateException.class, device::getCANBus);
  }

  @Test
  void concurrentTimeInterpolatableBufferInterpolatesAndExposesReadOnlyRanges() {
    ConcurrentTimeInterpolatableBuffer<Double> buffer =
        ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(1.0);

    assertTrue(buffer.getSample(0.0).isEmpty());
    assertTrue(buffer.getLatest().isEmpty());

    buffer.addSample(1.0, 10.0);
    buffer.addSample(2.0, 20.0);

    assertEquals(15.0, buffer.getSample(1.5).orElseThrow(), EPSILON);
    assertEquals(10.0, buffer.getSample(0.5).orElseThrow(), EPSILON);
    assertEquals(20.0, buffer.getSample(2.5).orElseThrow(), EPSILON);
    assertEquals(2.0, buffer.getLatest().orElseThrow().getKey(), EPSILON);

    NavigableMap<Double, Double> range = buffer.getSamplesInRange(1.0, true, 2.0, false);
    assertEquals(1, range.size());
    assertEquals(10.0, range.get(1.0), EPSILON);
    assertThrows(UnsupportedOperationException.class, () -> range.put(1.5, 15.0));
    assertTrue(buffer.getSamplesInRange(2.0, true, 1.0, true).isEmpty());
  }

  @Test
  void overrideSwitchesRejectInvalidIndexes() {
    OverrideSwitches switches = new OverrideSwitches(5);

    assertThrows(IllegalArgumentException.class, () -> switches.getDriverSwitch(-1));
    assertThrows(IllegalArgumentException.class, () -> switches.getDriverSwitch(3));
    assertThrows(IllegalArgumentException.class, () -> switches.getOperatorSwitch(-1));
    assertThrows(IllegalArgumentException.class, () -> switches.getOperatorSwitch(5));
  }
}
