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
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.subsystems.imu.Imu;
import frc.robot.util.RBSIEnum.Mode;
import frc.robot.util.VirtualSubsystem;
import org.littletonrobotics.junction.Logger;

public final class DriveOdometry extends VirtualSubsystem {

  // Declare the io and inputs
  private final Drive drive;
  private final Imu imu;
  private final Module[] modules;

  // Per-cycle cached objects (to avoid repeated allocations)
  private final SwerveModulePosition[] odomPositions = new SwerveModulePosition[4];

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
      // Ensure IMU inputs are fresh for this cycle
      final var imuInputs = imu.getInputs();

      // Drain per-module odometry queues ONCE per loop (this also refreshes motor signals)
      for (var module : modules) {
        module.periodic();
      }

      if (Constants.getMode() == Mode.SIM) {
        // SIMULATION: Keep sim pose buffer time-aligned, too
        final double now = Timer.getFPGATimestamp();
        drive.poseBufferAddSample(now, drive.getSimPose());
        drive.yawBuffersAddSample(now, drive.getSimYawRad(), drive.getSimYawRateRadPerSec());
        Logger.recordOutput("Drive/Pose", drive.getSimPose());
        return;
      }

      // Canonical timestamp queue from module[0]
      final double[] ts = modules[0].getOdometryTimestamps();
      final int n = (ts == null) ? 0 : ts.length;

      // Always keep yaw buffers “alive” even if no samples
      if (n == 0) {
        final double now = Timer.getFPGATimestamp();
        drive.yawBuffersAddSample(now, imuInputs.yawPositionRad, imuInputs.yawRateRadPerSec);
        drive.setGyroDisconnectedAlert(!imuInputs.connected);
        return;
      }

      // Cache module histories once
      final SwerveModulePosition[][] modHist = new SwerveModulePosition[4][];
      for (int m = 0; m < 4; m++) {
        modHist[m] = modules[m].getOdometryPositions();
      }

      // Determine YAW queue availability (everything exists and lines up)
      final boolean hasYawQueue =
          imuInputs.connected
              && imuInputs.odometryYawTimestamps != null
              && imuInputs.odometryYawPositionsRad != null
              && imuInputs.odometryYawTimestamps.length == imuInputs.odometryYawPositionsRad.length
              && imuInputs.odometryYawTimestamps.length > 0;

      final double[] yawTs = hasYawQueue ? imuInputs.odometryYawTimestamps : null;
      final double[] yawPos = hasYawQueue ? imuInputs.odometryYawPositionsRad : null;

      // Determine index alignment (cheap + deterministic)
      // We only trust index alignment if BOTH:
      //  - yaw has at least n samples
      //  - yawTs[i] ~= ts[i] for i in range (tight epsilon)
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
        // Single “now” sample once (not per replay)
        final double now = Timer.getFPGATimestamp();
        drive.yawBuffersAddSample(now, imuInputs.yawPositionRad, imuInputs.yawRateRadPerSec);
      }

      // Replay each odometry sample
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
            // Keep yaw buffers aligned to replay timeline
            drive.yawBuffersAddSampleIndexAligned(t, yawTs, yawPos, i);
          } else {
            yawRad = drive.yawBufferSampleOr(t, imuInputs.yawPositionRad);
          }
        }

        // Feed estimator at this historical timestamp
        drive.poseEstimatorUpdateWithTime(t, Rotation2d.fromRadians(yawRad), odomPositions);
        // Maintain pose history in SAME timebase as estimator
        drive.poseBufferAddSample(t, drive.poseEstimatorGetPose());
      }

      Logger.recordOutput("Drive/Pose", drive.poseEstimatorGetPose());
      drive.setGyroDisconnectedAlert(!imuInputs.connected);

    } finally {
      Drive.odometryLock.unlock();
    }
  }
}
