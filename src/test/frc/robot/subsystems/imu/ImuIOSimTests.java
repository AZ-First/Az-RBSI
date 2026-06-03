package frc.robot.subsystems.imu;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class ImuIOSimTests {
  private static final double EPSILON = 1e-9;

  @Test
  void odometrySamplesDrainOncePerUpdate() {
    ImuIOSim io = new ImuIOSim();
    ImuIO.ImuIOInputs inputs = new ImuIO.ImuIOInputs();

    io.simulationSetYawRad(1.0);
    io.updateInputs(inputs);
    assertEquals(1, inputs.odometryYawTimestamps.length);
    assertEquals(1, inputs.odometryYawPositionsRad.length);
    assertEquals(1.0, inputs.odometryYawPositionsRad[0], EPSILON);

    io.simulationSetYawRad(2.0);
    io.updateInputs(inputs);
    assertEquals(1, inputs.odometryYawTimestamps.length);
    assertEquals(1, inputs.odometryYawPositionsRad.length);
    assertEquals(2.0, inputs.odometryYawPositionsRad[0], EPSILON);
  }

  @Test
  void zeroYawClearsOldOdometrySamplesBeforeNextUpdate() {
    ImuIOSim io = new ImuIOSim();
    ImuIO.ImuIOInputs inputs = new ImuIO.ImuIOInputs();

    io.simulationSetYawRad(1.0);
    io.updateInputs(inputs);

    io.zeroYawRad(0.25);
    io.updateInputs(inputs);

    assertTrue(inputs.connected);
    assertEquals(0.25, inputs.yawPositionRad, EPSILON);
    assertEquals(1, inputs.odometryYawPositionsRad.length);
    assertEquals(0.25, inputs.odometryYawPositionsRad[0], EPSILON);
  }
}
