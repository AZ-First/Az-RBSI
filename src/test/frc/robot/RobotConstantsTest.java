package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;

import frc.robot.subsystems.drive.SwerveConstants;
import org.junit.jupiter.api.Test;

class RobotConstantsTest {
  @Test
  void robotDeviceCanBusesMatchSwerveConstants() {
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
}
