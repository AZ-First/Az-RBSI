package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.imu.Imu;
import frc.robot.subsystems.imu.ImuIOSim;
import frc.robot.util.MathUtil;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.command2.Command;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.simulation.DriverStationSim;

class DriveCommandsTests {
  private static final double EPSILON = 1e-9;

  @BeforeEach
  void setupHal() {
    HAL.initialize(500, 0);
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
  }

  @Test
  void linearVelocityAppliesDeadbandAndSquaresMagnitudeWithoutChangingDirection() {
    assertEquals(Translation2d.kZero, DriveCommands.getLinearVelocity(0.01, 0.01));

    Translation2d velocity = DriveCommands.getLinearVelocity(0.6, 0.8);
    assertEquals(1.0, velocity.getAngle().getCos() / 0.6, 1e-6);
    assertEquals(1.0, velocity.getAngle().getSin() / 0.8, 1e-6);
    assertEquals(1.0, velocity.getNorm(), EPSILON);
  }

  @Test
  void omegaAppliesDeadbandAndPreservesSignWhenSquared() {
    double scaled = MathUtil.applyDeadband(0.5, OperatorConstants.kJoystickDeadband);

    assertEquals(0.0, DriveCommands.getOmega(0.01), EPSILON);
    assertEquals(scaled * scaled, DriveCommands.getOmega(0.5), EPSILON);
    assertEquals(-(scaled * scaled), DriveCommands.getOmega(-0.5), EPSILON);
  }

  @Test
  void utilityCommandFactoriesReturnCommands() {
    Drive drive = new Drive(new Imu(new ImuIOSim()));

    Command stop = DriveCommands.stop(drive);
    Command stopWithX = DriveCommands.stopWithX(drive);
    Command brake = DriveCommands.setBrakeMode(drive, true);
    Command zero = DriveCommands.zeroHeadingForAlliance(drive);
    Command nudge = DriveCommands.robotRelativeNudge(drive, 0.1, 0.0, 0.0);

    assertNotNull(stop);
    assertNotNull(stopWithX);
    assertNotNull(brake);
    assertNotNull(zero);
    assertNotNull(nudge);
  }
}
