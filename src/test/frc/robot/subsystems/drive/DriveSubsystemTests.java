package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.subsystems.imu.Imu;
import frc.robot.subsystems.imu.ImuIOSim;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class DriveSubsystemTests {
  private static final double EPSILON = 1e-9;

  @BeforeEach
  void setupHal() {
    assertTrue(HAL.initialize(500, 0));
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
  }

  @Test
  void yawRateUsesWrappedAngleDifference() {
    double previous = Math.toRadians(179.0);
    double current = Math.toRadians(-179.0);

    assertEquals(Math.toRadians(2.0) / 0.02, Drive.yawRateRadPerSec(previous, current, 0.02), 1e-6);
    assertEquals(0.0, Drive.yawRateRadPerSec(previous, current, 0.0), EPSILON);
  }

  @Test
  void poseBufferAccessorsAreSafeWhenEmptyAndInterpolateWhenPopulated() {
    Drive drive = new Drive(new Imu(new ImuIOSim()));

    assertTrue(Double.isNaN(drive.getPoseBufferOldestTime()));
    assertTrue(Double.isNaN(drive.getPoseBufferNewestTime()));
    assertTrue(drive.getPoseAtTime(1.0).isEmpty());

    drive.poseBufferAddSample(1.0, new Pose2d(1.0, 0.0, Rotation2d.kZero));
    drive.poseBufferAddSample(2.0, new Pose2d(3.0, 0.0, Rotation2d.kZero));

    assertEquals(1.0, drive.getPoseBufferOldestTime(), EPSILON);
    assertEquals(2.0, drive.getPoseBufferNewestTime(), EPSILON);
    assertEquals(2.0, drive.getPoseAtTime(1.5).orElseThrow().getX(), EPSILON);
  }

  @Test
  void modulePeriodicUsesCommonOdometryPrefix() {
    ModuleIO fakeIo =
        new ModuleIO() {
          @Override
          public void updateInputs(ModuleIOInputs inputs) {
            inputs.driveConnected = true;
            inputs.turnConnected = true;
            inputs.turnEncoderConnected = true;
            inputs.odometryTimestamps = new double[] {1.0, 2.0, 3.0};
            inputs.odometryDrivePositionsRad = new double[] {4.0, 5.0};
            inputs.odometryTurnPositions =
                new Rotation2d[] {Rotation2d.kZero, Rotation2d.kCCW_Pi_2, Rotation2d.kPi};
          }
        };
    Module module = new Module(fakeIo, 0);

    assertDoesNotThrow(module::periodic);
    assertEquals(2, module.getOdometryPositions().length);
  }

  @Test
  void disabledCoastDoesNotCountBaselineSampleAsStationary() {
    Drive drive = new Drive(new Imu(new ImuIOSim()));
    SwerveModulePosition[] stationaryPositions =
        new SwerveModulePosition[] {
          new SwerveModulePosition(1.0, Rotation2d.kZero),
          new SwerveModulePosition(1.0, Rotation2d.kZero),
          new SwerveModulePosition(1.0, Rotation2d.kZero),
          new SwerveModulePosition(1.0, Rotation2d.kZero)
        };

    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    drive.updateDisabledCoastState(true, false, 1.0, 0.0, stationaryPositions);

    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    drive.updateDisabledCoastState(false, true, 1.02, 0.0, stationaryPositions);
    for (int i = 0; i < 9; i++) {
      drive.updateDisabledCoastState(false, true, 1.30 + i * 0.02, 0.0, stationaryPositions);
    }

    assertTrue(drive.isDisabledCoast(1.48));

    drive.updateDisabledCoastState(false, true, 1.50, 0.0, stationaryPositions);
    assertFalse(drive.isDisabledCoast(1.50));
  }
}
