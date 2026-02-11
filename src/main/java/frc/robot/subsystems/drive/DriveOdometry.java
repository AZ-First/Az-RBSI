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
  private final Drive drive;
  private final Imu imu;
  private final Module[] modules;

  // Scratch (no per-loop allocations)
  private final SwerveModulePosition[] odomPositions = new SwerveModulePosition[4];

  public DriveOdometry(Drive drive, Imu imu, Module[] modules) {
    this.drive = drive;
    this.imu = imu;
    this.modules = modules;
  }

  // Priority value for this virtual subsystem
  @Override
  protected int getPeriodPriority() {
    return -20;
  }

  @Override
  public void rbsiPeriodic() {
    Drive.odometryLock.lock();
    try {
      final var imuInputs = imu.getInputs();

      // Drain per-module odometry queues ONCE per loop (this also refreshes motor signals)
      for (var module : modules) {
        module.periodic(); // if you later split, this becomes module.updateInputs()
      }

      if (Constants.getMode() == Mode.SIM) {
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

      // Yaw queue availability
      final boolean hasYawQueue =
          imuInputs.connected
              && imuInputs.odometryYawTimestamps != null
              && imuInputs.odometryYawPositionsRad != null
              && imuInputs.odometryYawTimestamps.length == imuInputs.odometryYawPositionsRad.length
              && imuInputs.odometryYawTimestamps.length > 0;

      final double[] yawTs = hasYawQueue ? imuInputs.odometryYawTimestamps : null;
      final double[] yawPos = hasYawQueue ? imuInputs.odometryYawPositionsRad : null;

      // Determine index alignment (cheap + deterministic)
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

      // Replay
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

        double yawRad = imuInputs.yawPositionRad;
        if (hasYawQueue) {
          if (yawIndexAligned) {
            yawRad = yawPos[i];
            // Keep yaw buffers aligned to replay timeline (good for yaw-rate gate)
            drive.yawBuffersAddSampleIndexAligned(t, yawTs, yawPos, i);
          } else {
            yawRad = drive.yawBufferSampleOr(t, imuInputs.yawPositionRad);
          }
        }

        drive.poseEstimatorUpdateWithTime(t, Rotation2d.fromRadians(yawRad), odomPositions);
        drive.poseBufferAddSample(t, drive.poseEstimatorGetPose());
      }

      Logger.recordOutput("Drive/Pose", drive.poseEstimatorGetPose());
      drive.setGyroDisconnectedAlert(!imuInputs.connected);

    } finally {
      Drive.odometryLock.unlock();
    }
  }

  // /************************************************************************* */
  // /** Periodic function that is called each robot cycle by the command scheduler */
  // @Override
  // public void rbsiPeriodic() {
  //   odometryLock.lock();
  //   try {
  //     // Ensure IMU inputs are fresh for this cycle
  //     final var imuInputs = imu.getInputs();

  //     // Stop modules & log empty setpoint states if disabled
  //     if (DriverStation.isDisabled()) {
  //       for (var module : modules) {
  //         module.stop();
  //       }
  //       Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
  //       Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
  //     }

  //     // Drain per-module odometry queues for this cycle
  //     for (var module : modules) {
  //       module.periodic();
  //     }

  //     // -------- REAL ODOMETRY REPLAY (canonical timestamps from module[0]) --------
  //     if (Constants.getMode() != Mode.SIM) {
  //       final double[] ts = modules[0].getOdometryTimestamps();
  //       final int n = (ts == null) ? 0 : ts.length;

  //       // Cache per-module histories ONCE (avoid repeated getters in the loop)
  //       final SwerveModulePosition[][] modHist = new SwerveModulePosition[4][];
  //       for (int m = 0; m < 4; m++) {
  //         modHist[m] = modules[m].getOdometryPositions();
  //       }

  //       // Determine yaw queue availability
  //       final boolean hasYawQueue =
  //           imuInputs.connected
  //               && imuInputs.odometryYawTimestamps != null
  //               && imuInputs.odometryYawPositionsRad != null
  //               && imuInputs.odometryYawTimestamps.length
  //                   == imuInputs.odometryYawPositionsRad.length
  //               && imuInputs.odometryYawTimestamps.length > 0;

  //       final double[] yawTs = hasYawQueue ? imuInputs.odometryYawTimestamps : null;
  //       final double[] yawPos = hasYawQueue ? imuInputs.odometryYawPositionsRad : null;

  //       // If we have no module samples, still keep yaw buffers “alive” for gating callers
  //       if (n == 0) {
  //         final double now = Timer.getFPGATimestamp();
  //         yawBuffer.addSample(now, imuInputs.yawPositionRad);
  //         yawRateBuffer.addSample(now, imuInputs.yawRateRadPerSec);

  //         gyroDisconnectedAlert.set(!imuInputs.connected);
  //         return;
  //       }

  //       // Decide whether yaw queue is index-aligned with module[0] timestamps.
  //       // We only trust index alignment if BOTH:
  //       //  - yaw has at least n samples
  //       //  - yawTs[i] ~= ts[i] for i in range (tight epsilon)
  //       boolean yawIndexAligned = false;
  //       if (hasYawQueue && yawTs.length >= n) {
  //         yawIndexAligned = true;
  //         final double eps = 1e-3; // 1 ms
  //         for (int i = 0; i < n; i++) {
  //           if (Math.abs(yawTs[i] - ts[i]) > eps) {
  //             yawIndexAligned = false;
  //             break;
  //           }
  //         }
  //       }

  //       // If yaw is NOT index-aligned but we have a yaw queue, build yaw/yawRate buffers ONCE.
  //       if (hasYawQueue && !yawIndexAligned) {
  //         for (int k = 0; k < yawTs.length; k++) {
  //           yawBuffer.addSample(yawTs[k], yawPos[k]);
  //           if (k > 0) {
  //             final double dt = yawTs[k] - yawTs[k - 1];
  //             if (dt > 1e-6) {
  //               yawRateBuffer.addSample(yawTs[k], (yawPos[k] - yawPos[k - 1]) / dt);
  //             }
  //           }
  //         }
  //       }

  //       // If NO yaw queue, add a single “now” sample once (don’t do this per replay sample)
  //       if (!hasYawQueue) {
  //         final double now = Timer.getFPGATimestamp();
  //         yawBuffer.addSample(now, imuInputs.yawPositionRad);
  //         yawRateBuffer.addSample(now, imuInputs.yawRateRadPerSec);
  //       }

  //       // Replay each odometry sample
  //       for (int i = 0; i < n; i++) {
  //         final double t = ts[i];

  //         // Build module positions at this sample index (clamp defensively)
  //         for (int m = 0; m < 4; m++) {
  //           final SwerveModulePosition[] hist = modHist[m];
  //           if (hist == null || hist.length == 0) {
  //             odomPositions[m] = modules[m].getPosition();
  //           } else if (i < hist.length) {
  //             odomPositions[m] = hist[i];
  //           } else {
  //             odomPositions[m] = hist[hist.length - 1];
  //           }
  //         }

  //         // Determine yaw at this timestamp
  //         double yawRad = imuInputs.yawPositionRad; // fallback

  //         if (hasYawQueue) {
  //           if (yawIndexAligned) {
  //             yawRad = yawPos[i];

  //             // Keep yaw/yawRate buffers updated in odometry timebase (good for yaw-gate)
  //             yawBuffer.addSample(t, yawRad);
  //             if (i > 0) {
  //               final double dt = yawTs[i] - yawTs[i - 1];
  //               if (dt > 1e-6) {
  //                 yawRateBuffer.addSample(t, (yawPos[i] - yawPos[i - 1]) / dt);
  //               }
  //             }
  //           } else {
  //             // yawBuffer was pre-filled above; interpolate here
  //             yawRad = yawBuffer.getSample(t).orElse(imuInputs.yawPositionRad);
  //           }
  //         }

  //         // Feed estimator at this historical timestamp
  //         m_PoseEstimator.updateWithTime(t, Rotation2d.fromRadians(yawRad), odomPositions);

  //         // Maintain pose history in SAME timebase as estimator
  //         poseBuffer.addSample(t, m_PoseEstimator.getEstimatedPosition());
  //       }

  //       Logger.recordOutput("Drive/Pose", m_PoseEstimator.getEstimatedPosition());
  //       gyroDisconnectedAlert.set(!imuInputs.connected);
  //       return;

  //     } else {

  //       // SIMULATION: Keep sim pose buffer time-aligned, too
  //       double now = Timer.getFPGATimestamp();
  //       poseBuffer.addSample(now, simPhysics.getPose());
  //       yawBuffer.addSample(now, simPhysics.getYaw().getRadians());
  //       yawRateBuffer.addSample(now, simPhysics.getOmegaRadPerSec());

  //       Logger.recordOutput("Drive/Pose", simPhysics.getPose());
  //       gyroDisconnectedAlert.set(false);
  //     }
  //   } finally {
  //     odometryLock.unlock();
  //   }
  // }

}
