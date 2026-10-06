package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.subsystems.drive.SwerveConstants;
import java.util.Arrays;
import org.junit.jupiter.api.Test;

class RobotConstantsTest {
  @Test
  void robotDeviceCanBusesMatchSwerveConstants() {
    assertEquals(SwerveConstants.kPigeonCanbus, Constants.RobotDevices.PIGEON.getBus());
    assertEquals(SwerveConstants.kFLDriveCanbus, Constants.RobotDevices.FL_DRIVE.getBus());
    assertEquals(SwerveConstants.kFLSteerCanbus, Constants.RobotDevices.FL_ROTATION.getBus());
    assertEquals(SwerveConstants.kFLEncoderCanbus, Constants.RobotDevices.FL_CANCODER.getBus());

    assertEquals(SwerveConstants.kFRDriveCanbus, Constants.RobotDevices.FR_DRIVE.getBus());
    assertEquals(SwerveConstants.kFRSteerCanbus, Constants.RobotDevices.FR_ROTATION.getBus());
    assertEquals(SwerveConstants.kFREncoderCanbus, Constants.RobotDevices.FR_CANCODER.getBus());

    assertEquals(SwerveConstants.kBLDriveCanbus, Constants.RobotDevices.BL_DRIVE.getBus());
    assertEquals(SwerveConstants.kBLSteerCanbus, Constants.RobotDevices.BL_ROTATION.getBus());
    assertEquals(SwerveConstants.kBLEncoderCanbus, Constants.RobotDevices.BL_CANCODER.getBus());

    assertEquals(SwerveConstants.kBRDriveCanbus, Constants.RobotDevices.BR_DRIVE.getBus());
    assertEquals(SwerveConstants.kBRSteerCanbus, Constants.RobotDevices.BR_ROTATION.getBus());
    assertEquals(SwerveConstants.kBREncoderCanbus, Constants.RobotDevices.BR_CANCODER.getBus());
  }

  @Test
  void eachModuleHasOneRegisteredCANBusForAllThreeDevices() {
    for (int module = 0; module < 4; module++) {
      String moduleBus = SwerveConstants.moduleCANBus(module);
      assertTrue(Arrays.asList(Constants.CANBuses.ALL).contains(moduleBus));
      switch (module) {
        case 0 -> {
          assertEquals(Constants.DrivebaseConstants.kFLCANBus, moduleBus);
          assertEquals(moduleBus, SwerveConstants.kFLDriveCanbus);
          assertEquals(moduleBus, SwerveConstants.kFLSteerCanbus);
          assertEquals(moduleBus, SwerveConstants.kFLEncoderCanbus);
        }
        case 1 -> {
          assertEquals(Constants.DrivebaseConstants.kFRCANBus, moduleBus);
          assertEquals(moduleBus, SwerveConstants.kFRDriveCanbus);
          assertEquals(moduleBus, SwerveConstants.kFRSteerCanbus);
          assertEquals(moduleBus, SwerveConstants.kFREncoderCanbus);
        }
        case 2 -> {
          assertEquals(Constants.DrivebaseConstants.kBLCANBus, moduleBus);
          assertEquals(moduleBus, SwerveConstants.kBLDriveCanbus);
          assertEquals(moduleBus, SwerveConstants.kBLSteerCanbus);
          assertEquals(moduleBus, SwerveConstants.kBLEncoderCanbus);
        }
        case 3 -> {
          assertEquals(Constants.DrivebaseConstants.kBRCANBus, moduleBus);
          assertEquals(moduleBus, SwerveConstants.kBRDriveCanbus);
          assertEquals(moduleBus, SwerveConstants.kBRSteerCanbus);
          assertEquals(moduleBus, SwerveConstants.kBREncoderCanbus);
        }
        default -> throw new AssertionError();
      }
    }
    assertTrue(Arrays.asList(Constants.CANBuses.ALL).contains(SwerveConstants.kPigeonCanbus));
    assertThrows(IllegalArgumentException.class, () -> SwerveConstants.moduleCANBus(4));
  }

  @Test
  void systemcorePortsAreExplicit() {
    assertEquals(0, Constants.CANBuses.revPort(Constants.CANBuses.SYSTEMCORE_0));
    assertEquals(4, Constants.CANBuses.revPort(Constants.CANBuses.SYSTEMCORE_4));
    assertThrows(IllegalArgumentException.class, () -> Constants.CANBuses.revPort("DriveTrain"));
  }

  @Test
  void fieldLayoutConstantsAreInternallyConsistent() {
    assertEquals(FieldConstants.defaultAprilTagType.getLayout(), FieldConstants.aprilTagLayout);
    assertEquals(
        FieldConstants.defaultAprilTagType.getLayout().getFieldLength(),
        FieldConstants.fieldLength,
        1e-9);
    assertEquals(
        FieldConstants.defaultAprilTagType.getLayout().getFieldWidth(),
        FieldConstants.fieldWidth,
        1e-9);
    assertEquals(
        FieldConstants.defaultAprilTagType.getLayout().getTags().size(),
        FieldConstants.aprilTagCount);
    assertNotNull(FieldConstants.defaultAprilTagType.getLayoutString());
  }

  @Test
  void selectedTunableConstantsAreInUsableRanges() {
    assertPositive(Constants.OperatorConstants.kRobotRelativeNudgeSpeedMetersPerSec);
    assertPositive(Constants.SensorConstants.kRioAccelerometerSampleRateHz);

    // Commands V3 deferred: assertPositive(Constants.DrivebaseConstants.kSysIdPreRunStopSecs);
    assertPositive(Constants.DrivebaseConstants.kFeedforwardCharacterizationStartDelaySecs);
    assertPositive(Constants.DrivebaseConstants.kFeedforwardCharacterizationRampRateVoltsPerSec);
    assertPositive(Constants.DrivebaseConstants.kWheelRadiusCharacterizationStartDelaySecs);
    assertPositive(Constants.DrivebaseConstants.kWheelRadiusCharacterizationMaxVelocityRadPerSec);
    assertPositive(Constants.DrivebaseConstants.kWheelRadiusCharacterizationRampRateRadPerSecSq);
    assertPositive(Constants.DrivebaseConstants.kDisabledCoastMinSeconds);
    assertPositive(Constants.DrivebaseConstants.kDisabledVisionCoastBlendAlpha);

    assertPositive(Constants.FlywheelConstants.kMaxVoltage);
    assertPositive(Constants.FlywheelConstants.kMotionMagicAccelerationRotPerSecSq);
    assertPositive(Constants.FlywheelConstants.kMotionMagicJerkRotPerSecCubed);
    assertPositive(Constants.FlywheelConstants.kSimGearing);
    assertPositive(Constants.FlywheelConstants.kSimMomentOfInertiaKgMetersSq);

    assertEquals(
        Math.min(
            Constants.DrivebaseConstants.kDisabledVisionBlendAlpha,
            Constants.DrivebaseConstants.kDisabledVisionCoastBlendAlpha),
        Constants.DrivebaseConstants.kDisabledVisionCoastBlendAlpha,
        1e-9);
  }

  @Test
  void constantsDoNotExposeLegacyAliasNames() {
    assertMissing(Constants.class, "loopPeriodSecs");
    assertMissing(Constants.class, "tuningMode");
    assertMissing(Constants.class, "G_TO_MPS2");

    assertMissing(Constants.RobotConstants.class, "kRobotMass");
    assertMissing(Constants.RobotConstants.class, "kRobotMOI");
    assertMissing(Constants.RobotConstants.class, "kWheelCOF");
    assertMissing(Constants.RobotConstants.class, "kMaxWheelTorque");

    assertMissing(Constants.PowerConstants.class, "kPDMType");
    assertMissing(Constants.PowerConstants.class, "kPDMCANid");
    assertMissing(Constants.PowerConstants.class, "kTotalMaxCurrent");
    assertMissing(Constants.PowerConstants.class, "kMotorPortMaxCurrent");
    assertMissing(Constants.PowerConstants.class, "kVoltageWarning");

    assertMissing(Constants.DrivebaseConstants.class, "kMaxLinearSpeed");
    assertMissing(Constants.DrivebaseConstants.class, "kPStrafe");
    assertMissing(Constants.DrivebaseConstants.class, "kPSPin");
    assertMissing(Constants.DrivebaseConstants.class, "kWheelLockTime");
    assertMissing(Constants.DrivebaseConstants.class, "kHistorySize");

    assertMissing(Constants.FlywheelConstants.class, "kFlywheelGearRatio");
    assertMissing(Constants.FlywheelConstants.class, "kSreal");
    assertMissing(Constants.FlywheelConstants.class, "kPsim");

    assertMissing(Constants.VisionConstants.class, "maxAmbiguity");
    assertMissing(Constants.VisionConstants.class, "linearStdDevBaseline");

    assertMissing(Constants.DeployConstants.class, "yagslDir");

    assertMissing(Constants.OperatorConstants.class, "kDeadband");
    assertMissing(Constants.OperatorConstants.class, "kTurnConstant");
    assertMissing(Constants.OperatorConstants.class, "kJoystickSlewLimit");
  }

  private static void assertMissing(Class<?> constantsClass, String fieldName) {
    assertThrows(NoSuchFieldException.class, () -> constantsClass.getDeclaredField(fieldName));
  }

  private static void assertPositive(double value) {
    org.junit.jupiter.api.Assertions.assertTrue(value > 0.0);
  }
}
