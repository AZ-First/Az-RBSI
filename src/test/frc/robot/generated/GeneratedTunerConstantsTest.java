package frc.robot.generated;

import static org.junit.jupiter.api.Assertions.assertEquals;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.util.Units;
import org.junit.jupiter.api.Test;

class GeneratedTunerConstantsTest {
  private static final double EPSILON = 1e-9;
  private static final double TEMPLATE_XY_METERS = Units.inchesToMeters(10.0);
  private static final double TEMPLATE_DRIVE_GEAR_RATIO = 6.746031746031747;
  private static final double TEMPLATE_STEER_GEAR_RATIO = 21.428571428571427;
  private static final double TEMPLATE_COUPLING_RATIO = 3.5714285714285716;
  private static final double TEMPLATE_WHEEL_RADIUS_METERS = Units.inchesToMeters(2.0);

  @Test
  void templateTunerConstantsUseZeroOffsetsAndTenInchModuleLocations() {
    assertTemplateGeometry(
        COMPBOTTunerConstants.FrontLeft,
        COMPBOTTunerConstants.FrontRight,
        COMPBOTTunerConstants.BackLeft,
        COMPBOTTunerConstants.BackRight);
    assertTemplateGeometry(
        DEVBOT1TunerConstants.FrontLeft,
        DEVBOT1TunerConstants.FrontRight,
        DEVBOT1TunerConstants.BackLeft,
        DEVBOT1TunerConstants.BackRight);
    assertTemplateGeometry(
        DEVBOT2TunerConstants.FrontLeft,
        DEVBOT2TunerConstants.FrontRight,
        DEVBOT2TunerConstants.BackLeft,
        DEVBOT2TunerConstants.BackRight);
  }

  @Test
  void templateTunerConstantsUseMatchingRatios() {
    assertTemplateRatios(COMPBOTTunerConstants.FrontLeft);
    assertTemplateRatios(DEVBOT1TunerConstants.FrontLeft);
    assertTemplateRatios(DEVBOT2TunerConstants.FrontLeft);
  }

  private static void assertTemplateGeometry(
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          frontLeft,
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          frontRight,
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          backLeft,
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          backRight) {
    assertModule(frontLeft, TEMPLATE_XY_METERS, TEMPLATE_XY_METERS);
    assertModule(frontRight, TEMPLATE_XY_METERS, -TEMPLATE_XY_METERS);
    assertModule(backLeft, -TEMPLATE_XY_METERS, TEMPLATE_XY_METERS);
    assertModule(backRight, -TEMPLATE_XY_METERS, -TEMPLATE_XY_METERS);
  }

  private static void assertModule(
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          module,
      double expectedX,
      double expectedY) {
    assertEquals(0.0, module.EncoderOffset, EPSILON);
    assertEquals(expectedX, module.LocationX, EPSILON);
    assertEquals(expectedY, module.LocationY, EPSILON);
  }

  private static void assertTemplateRatios(
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          module) {
    assertEquals(TEMPLATE_DRIVE_GEAR_RATIO, module.DriveMotorGearRatio, EPSILON);
    assertEquals(TEMPLATE_STEER_GEAR_RATIO, module.SteerMotorGearRatio, EPSILON);
    assertEquals(TEMPLATE_COUPLING_RATIO, module.CouplingGearRatio, EPSILON);
    assertEquals(TEMPLATE_WHEEL_RADIUS_METERS, module.WheelRadius, EPSILON);
  }
}
