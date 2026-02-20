// Copyright (c) 2024-2026 Az-FIRST
// http://github.com/AZ-First
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
//
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.subsystems.imu.Imu;
import frc.robot.util.RBSIEnum.Mode;
import frc.robot.util.TimeUtil;
import frc.robot.util.VirtualSubsystem;
import org.littletonrobotics.junction.Logger;

public final class DriveOdometry extends VirtualSubsystem {

  // Declare the io and inputs
  private final Drive drive;
  private final Imu imu;
  private final Module[] modules;

  // Per-cycle cached objects (to avoid repeated allocations)
  private final SwerveModulePosition[] odomPositions = new SwerveModulePosition[4];

  // Checking whether this is a REPLAY
  private boolean isReplayActive = Logger.hasReplaySource();

  /** Constructor */
  public DriveOdometry(Drive drive, Imu imu, Module[] modules) {
    this.drive = drive;
    this.imu = imu;
    this.modules = modules;
  }

  /**
   * Priority value for this virtual subsystem
   *
   * <p>See `frc.robot.util.VirtualSubsystem` for a description of the suggested values for various
   * virtual subsystems.
   */
  @Override
  protected int getPeriodPriority() {
    return -20;
  }

  /** Periodic function to read inputs */
  @Override
  public void rbsiPeriodic() {

    Drive.odometryLock.lock();
    try {
      final var imuInputs = imu.getInputs();

      // Drain per-module odometry queues ONCE per loop (refresh signals).
      for (var module : modules) {
        module.periodic();
      }

      // ----------------------------------------------------------------------
      // Pure SIM (not replaying a log): use sim pose/yaw
      // ----------------------------------------------------------------------
      if (Constants.isPureSim()) {
        final double now = TimeUtil.now();

        // Keep buffers alive
        drive.poseBufferAddSample(now, drive.getSimPose());
        drive.yawBuffersAddSample(now, drive.getSimYawRad(), drive.getSimYawRateRadPerSec());

        // Coast state uses "now" + current module positions
        drive.updateDisabledCoastState(
            DriverStation.isEnabled(),
            DriverStation.isDisabled(),
            now,
            drive.getSimYawRateRadPerSec(),
            drive.getModulePositions());

        return;
      }

      // ----------------------------------------------------------------------
      // DISABLED (REAL only): minimal ticking — keep buffers alive, do NOT integrate module deltas.
      // (If you want replay integration while disabled, this branch is already !isReplayActive.)
      // ----------------------------------------------------------------------
      if (DriverStation.isDisabled() && !isReplayActive) {
        final double now = TimeUtil.now();

        // keep yaw buffers alive
        if (imuInputs.connected) {
          drive.yawBuffersAddSample(now, imuInputs.yawPositionRad, imuInputs.yawRateRadPerSec);
        }

        // Coast state from "now" + current module positions
        drive.updateDisabledCoastState(
            DriverStation.isEnabled(),
            DriverStation.isDisabled(),
            now,
            imuInputs.yawRateRadPerSec,
            drive.getModulePositions());

        // keep pose buffer alive with the *current estimator pose*
        drive.poseBufferAddSample(now, drive.getPose());
        drive.setGyroDisconnectedAlert(!imuInputs.connected);
        return;
      }

      // ----------------------------------------------------------------------
      // Canonical timestamp queue from module[0]
      // ----------------------------------------------------------------------
      final double[] ts = modules[0].getOdometryTimestamps();
      final int n = (ts == null) ? 0 : ts.length;

      // Always keep yaw buffers “alive” even if no samples
      if (n == 0) {
        if (Constants.getMode() != Mode.REPLAY) {
          final double now = TimeUtil.now();
          drive.yawBuffersAddSample(now, imuInputs.yawPositionRad, imuInputs.yawRateRadPerSec);

          // Coast state update (no per-sample positions available; use current)
          drive.updateDisabledCoastState(
              DriverStation.isEnabled(),
              DriverStation.isDisabled(),
              now,
              imuInputs.yawRateRadPerSec,
              drive.getModulePositions());
        }
        drive.setGyroDisconnectedAlert(!imuInputs.connected);
        return;
      }

      // Cache module histories once
      final SwerveModulePosition[][] modHist = new SwerveModulePosition[4][];
      for (int m = 0; m < 4; m++) {
        modHist[m] = modules[m].getOdometryPositions();
      }

      // ----------------------------------------------------------------------
      // Determine YAW queue availability (everything exists and lines up)
      // ----------------------------------------------------------------------
      final boolean hasYawQueue =
          imuInputs.connected
              && imuInputs.odometryYawTimestamps != null
              && imuInputs.odometryYawPositionsRad != null
              && imuInputs.odometryYawTimestamps.length == imuInputs.odometryYawPositionsRad.length
              && imuInputs.odometryYawTimestamps.length > 0;

      final double[] yawTs = hasYawQueue ? imuInputs.odometryYawTimestamps : null;
      final double[] yawPos = hasYawQueue ? imuInputs.odometryYawPositionsRad : null;

      // Determine index alignment
      boolean yawIndexAligned = false;
      if (hasYawQueue && yawTs.length >= n) {
        yawIndexAligned = true;
        final double eps = 1e-3; // 1ms
        for (int i = 0; i < n; i++) {
          if (Math.abs(yawTs[i] - ts[i]) > eps) {
            yawIndexAligned = false;
            break;
          }
        }
      }

      // If yaw not aligned, pre-fill yaw buffers once and interpolate later
      if (hasYawQueue && !yawIndexAligned) {
        drive.yawBuffersFillFromQueue(yawTs, yawPos);
      } else if (!hasYawQueue) {
        final double now = TimeUtil.now();
        drive.yawBuffersAddSample(now, imuInputs.yawPositionRad, imuInputs.yawRateRadPerSec);
      }

      // ----------------------------------------------------------------------
      // Replay each odometry sample
      // ----------------------------------------------------------------------
      final double[] lastDist = new double[4];
      boolean haveLastDist = false;

      for (int i = 0; i < n; i++) {
        final double t = ts[i];

        // Build module positions at sample i (clamp defensively)
        for (int m = 0; m < 4; m++) {
          final SwerveModulePosition[] hist = modHist[m];
          if (hist == null || hist.length == 0) {
            odomPositions[m] = modules[m].getPosition();
          } else if (i < hist.length) {
            odomPositions[m] = hist[i];
          } else {
            odomPositions[m] = hist[hist.length - 1];
          }
        }

        // Determine yaw at this timestamp
        double yawRad = imuInputs.yawPositionRad;
        if (hasYawQueue) {
          if (yawIndexAligned) {
            yawRad = yawPos[i];
            drive.yawBuffersAddSampleIndexAligned(t, yawTs, yawPos, i);
          } else {
            yawRad = drive.yawBufferSampleOr(t, imuInputs.yawPositionRad);
          }
        }

        // Coast state update IN REPLAY TIMEBASE
        // Yaw rate: if you have a buffered rate, use it; otherwise imuInputs.yawRateRadPerSec is
        // ok.
        drive.updateDisabledCoastState(
            DriverStation.isEnabled(),
            DriverStation.isDisabled(),
            t,
            imuInputs.yawRateRadPerSec,
            odomPositions);

        // Debugging
        Logger.recordOutput("Odometry/Debug/timestamp", t);
        Logger.recordOutput("Odometry/Debug/now", TimeUtil.now());
        if (i > 0) {
          Logger.recordOutput("Odometry/Debug/timeNowDiff", t - ts[i - 1]);
        }
        Logger.recordOutput("Odometry/Debug/replay_t", t);
        Logger.recordOutput("Odometry/Debug/replay_yawRad", yawRad);

        // Module distance deltas (valid within batch)
        for (int m = 0; m < 4; m++) {
          final SwerveModulePosition pos = odomPositions[m];
          final double dist = pos.distanceMeters;

          Logger.recordOutput("Odometry/Debug/mod" + m + "_distanceMeters", dist);
          Logger.recordOutput("Odometry/Debug/mod" + m + "_angleRad", pos.angle.getRadians());

          if (haveLastDist) {
            final double delta = dist - lastDist[m];
            Logger.recordOutput("Odometry/Debug/mod" + m + "_deltaMeters", delta);
          }

          lastDist[m] = dist;
        }
        haveLastDist = true;

        // Feed estimator at this historical timestamp
        drive.poseEstimatorUpdateWithTime(t, Rotation2d.fromRadians(yawRad), odomPositions);

        // Maintain pose history in SAME timebase as estimator
        drive.poseBufferAddSample(t, drive.getPose());
      }

      drive.setGyroDisconnectedAlert(!imuInputs.connected);

    } finally {

      Logger.recordOutput("Odometry/Robot", drive.getPose());

      Drive.odometryLock.unlock();
    }
  }
}
