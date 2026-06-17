package frc.robot.subsystems.accelerometer;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;

import org.junit.jupiter.api.Test;

class RioAccelIOTests {
  @Test
  void noopReportsDisconnectedZeroAcceleration() {
    RioAccelIO.Inputs inputs = new RioAccelIO.Inputs();

    RioAccelIO.noop().updateInputs(inputs);

    assertFalse(inputs.connected);
    assertEquals(0L, inputs.timestampNs);
    assertEquals(0.0, inputs.xG);
    assertEquals(0.0, inputs.yG);
    assertEquals(0.0, inputs.zG);
  }
}
