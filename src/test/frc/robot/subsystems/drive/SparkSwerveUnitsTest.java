package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Rotation2d;

class SparkSwerveUnitsTest {
  private static final double EPSILON = 1e-9;

  @Test
  void driveEncoderAndControllerUseInverseUnits() {
    double wheelRadiansPerSecond = 5.0;
    double motorRpm = SparkSwerveUnits.driveVelocityRpm(wheelRadiansPerSecond);

    assertEquals(
        wheelRadiansPerSecond, SparkSwerveUnits.driveVelocityRadiansPerSecond(motorRpm), EPSILON);
    assertEquals(
        wheelRadiansPerSecond * 60.0, SparkSwerveUnits.drivePositionRadians(motorRpm), EPSILON);
    assertEquals(
        DRIVE_GAIN * wheelRadiansPerSecond,
        SparkSwerveUnits.driveVelocityGain(DRIVE_GAIN) * motorRpm,
        EPSILON);
  }

  @Test
  void turnSetpointWrapsInAbsoluteEncoderRotations() {
    Rotation2d zeroRotation = Rotation2d.fromDegrees(20.0);
    double setpoint =
        SparkSwerveUnits.turnSetpointRotations(Rotation2d.fromDegrees(350.0), zeroRotation);

    assertEquals(10.0 / 360.0, setpoint, EPSILON);
    assertEquals(
        Rotation2d.fromDegrees(-10.0).getRadians(),
        SparkSwerveUnits.turnPosition(setpoint, zeroRotation).getRadians(),
        EPSILON);
    assertEquals(2.0 * Math.PI, SparkSwerveUnits.turnVelocityRadiansPerSecond(60.0), EPSILON);
    assertEquals(
        STEER_GAIN * Math.PI, SparkSwerveUnits.turnPositionGain(STEER_GAIN) * 0.5, EPSILON);
  }

  private static final double DRIVE_GAIN = 0.7;
  private static final double STEER_GAIN = 100.0;
}
