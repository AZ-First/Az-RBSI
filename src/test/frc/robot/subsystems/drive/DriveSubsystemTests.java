package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.subsystems.imu.Imu;
import frc.robot.subsystems.imu.ImuIOSim;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.command3.sysid.SysIdRoutine;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.simulation.DriverStationSim;

class DriveSubsystemTests {
  private static final double EPSILON = 1e-9;

  @BeforeEach
  void setupHal() {
    assertTrue(HAL.initialize());
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

    drive.poseBufferAddSample(1.0, new Pose2d(1.0, 0.0, Rotation2d.ZERO));
    drive.poseBufferAddSample(2.0, new Pose2d(3.0, 0.0, Rotation2d.ZERO));

    assertEquals(1.0, drive.getPoseBufferOldestTime(), EPSILON);
    assertEquals(2.0, drive.getPoseBufferNewestTime(), EPSILON);
    assertEquals(2.0, drive.getPoseAtTime(1.5).orElseThrow().getX(), EPSILON);
  }

  @Test
  void sysIdCommandsRequireTheDriveMechanism() {
    Drive drive = new Drive(new Imu(new ImuIOSim()));

    var quasistatic = drive.sysIdQuasistatic(SysIdRoutine.Direction.FORWARD);
    var dynamic = drive.sysIdDynamic(SysIdRoutine.Direction.REVERSE);

    assertTrue(quasistatic.requires(drive));
    assertTrue(dynamic.requires(drive));
    assertEquals("Drive SysId Forward Quasistatic", quasistatic.name());
    assertEquals("Drive SysId Reverse Dynamic", dynamic.name());
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
                new Rotation2d[] {
                  Rotation2d.ZERO, Rotation2d.fromDegrees(90.0), Rotation2d.fromDegrees(180.0)
                };
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
          new SwerveModulePosition(1.0, Rotation2d.ZERO),
          new SwerveModulePosition(1.0, Rotation2d.ZERO),
          new SwerveModulePosition(1.0, Rotation2d.ZERO),
          new SwerveModulePosition(1.0, Rotation2d.ZERO)
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
  // Commands V3 deferred: PathPlanner start-pose policy tests
  //   @Test
  //   void pathPlannerStartResetsWhenVisionIsMissingOrEstimatorIsUninitialized() {
  //     Pose2d pathStart = new Pose2d(4.0, 6.0, Rotation2d.kZero);
  //
  //     assertEquals(
  //         Drive.PathPlannerStartAction.RESET_TO_PATH_START,
  //         Drive.determinePathPlannerStartAction(
  //             new Pose2d(6.0, 6.0, Rotation2d.kZero), pathStart, false, 0.5));
  //     assertEquals(
  //         Drive.PathPlannerStartAction.RESET_TO_PATH_START,
  //         Drive.determinePathPlannerStartAction(Pose2d.kZero, pathStart, true, 0.5));
  //   }
  //
  //   @Test
  //   void pathPlannerStartUsesNearbyVisionAndBlocksDistantVision() {
  //     Pose2d pathStart = new Pose2d(4.0, 6.0, Rotation2d.kZero);
  //
  //     assertEquals(
  //         Drive.PathPlannerStartAction.USE_VISION_POSE,
  //         Drive.determinePathPlannerStartAction(
  //             new Pose2d(4.4, 6.0, Rotation2d.kZero), pathStart, true, 0.5));
  //     assertEquals(
  //         Drive.PathPlannerStartAction.BLOCK_AUTO,
  //         Drive.determinePathPlannerStartAction(
  //             new Pose2d(4.6, 6.0, Rotation2d.kZero), pathStart, true, 0.5));
  //   }
  // End deferred integration.
}
