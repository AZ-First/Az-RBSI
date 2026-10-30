package frc.robot.subsystems.accelerometer;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.hardware.imu.OnboardIMU;
import org.wpilib.hardware.imu.OnboardIMU.MountOrientation;

class ControllerAccelIOTests {
  @Test
  void noopReportsDisconnectedZeroAcceleration() {
    ControllerAccelIO.ControllerAccelIOInputs inputs =
        new ControllerAccelIO.ControllerAccelIOInputs();

    ControllerAccelIO.noop().updateInputs(inputs);

    assertFalse(inputs.connected);
    assertEquals(0L, inputs.timestampNs);
    assertEquals(0.0, inputs.xMetersPerSecondSq);
    assertEquals(0.0, inputs.yMetersPerSecondSq);
    assertEquals(0.0, inputs.zMetersPerSecondSq);
  }

  @Test
  void onboardReadingsStayInMetersPerSecondSquared() {
    assertTrue(HAL.initialize());
    OnboardIMU imu =
        new OnboardIMU(MountOrientation.FLAT) {
          @Override
          public double getAccelX() {
            return 1.25;
          }

          @Override
          public double getAccelY() {
            return -2.5;
          }

          @Override
          public double getAccelZ() {
            return 9.8;
          }
        };
    ControllerAccelIO.ControllerAccelIOInputs inputs =
        new ControllerAccelIO.ControllerAccelIOInputs();

    new ControllerAccelIOOnboard(imu).updateInputs(inputs);

    assertTrue(inputs.connected);
    assertEquals(1.25, inputs.xMetersPerSecondSq);
    assertEquals(-2.5, inputs.yMetersPerSecondSq);
    assertEquals(9.8, inputs.zMetersPerSecondSq);
    assertTrue(inputs.timestampNs >= 0L);
  }
}
