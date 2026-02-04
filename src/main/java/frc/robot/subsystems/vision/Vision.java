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

package frc.robot.subsystems.vision;

import static frc.robot.Constants.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import frc.robot.util.VirtualSubsystem;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.Set;
import java.util.concurrent.atomic.AtomicReference;
import org.littletonrobotics.junction.Logger;

public class Vision extends VirtualSubsystem {

  @FunctionalInterface
  public interface VisionConsumer {
    void accept(Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> stdDevs);
  }

  private record TimedEstimate(Pose2d pose, double ts, Matrix<N3, N1> stdDevs) {}

  private final VisionConsumer consumer;
  private final Drive drive;
  private long lastSeenPoseResetEpoch = -1;

  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;

  private final Constants.Cameras.CameraConfig[] camConfigs = Constants.Cameras.ALL;

  // --- Per-camera monotonic gate ---
  private final double[] lastAcceptedTsPerCam;

  // --- Pose reset gate ---
  private volatile double lastPoseResetTimestamp = Double.NEGATIVE_INFINITY;

  // --- Smoothing buffer (recent fused estimates) ---
  private final ArrayDeque<TimedEstimate> fusedBuffer = new ArrayDeque<>();
  private final double smoothWindowSec = 0.25;
  private final int smoothMaxSize = 12;

  // --- Trusted tags configuration (swappable per event/field) ---
  private final AtomicReference<Set<Integer>> trustedTags = new AtomicReference<>(Set.of());
  private volatile boolean requireTrustedTag = false;

  // Scale factors applied based on fraction of trusted tags in an observation
  private volatile double trustedTagStdDevScale = 0.70; // < 1 => more trusted
  private volatile double untrustedTagStdDevScale = 1.40; // > 1 => less trusted

  // --- Optional 254-style yaw gate for single-tag ---
  private volatile boolean enableSingleTagYawGate = true;
  private volatile double yawGateLookbackSec = 0.30;
  private volatile double yawGateLimitRadPerSec = 5.0;

  public Vision(Drive drive, VisionConsumer consumer, VisionIO... io) {
    this.drive = drive;
    this.consumer = consumer;
    this.io = io;

    this.inputs = new VisionIOInputsAutoLogged[io.length];
    for (int i = 0; i < io.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }

    this.lastAcceptedTsPerCam = new double[io.length];
    Arrays.fill(lastAcceptedTsPerCam, Double.NEGATIVE_INFINITY);

    // Log robot->camera transforms if available
    int n = Math.min(camConfigs.length, io.length);
    for (int i = 0; i < n; i++) {
      Logger.recordOutput("Vision/RobotToCamera" + i, camConfigs[i].robotToCamera());
    }
  }

  // -------- Runtime configuration hooks --------

  /** Call when you reset odometry (e.g. auto start, manual reset, etc). */
  public void resetPoseGate(double fpgaNowSeconds) {
    lastPoseResetTimestamp = fpgaNowSeconds;
    fusedBuffer.clear();
    Arrays.fill(lastAcceptedTsPerCam, Double.NEGATIVE_INFINITY);
  }

  /** Swap trusted tag set per event/field without redeploy. */
  public void setTrustedTags(Set<Integer> tags) {
    trustedTags.set(Set.copyOf(tags));
  }

  public void setRequireTrustedTag(boolean require) {
    requireTrustedTag = require;
  }

  public void setTrustedTagStdDevScales(double trustedScale, double untrustedScale) {
    trustedTagStdDevScale = trustedScale;
    untrustedTagStdDevScale = untrustedScale;
  }

  public void setSingleTagYawGate(boolean enabled, double lookbackSec, double limitRadPerSec) {
    enableSingleTagYawGate = enabled;
    yawGateLookbackSec = lookbackSec;
    yawGateLimitRadPerSec = limitRadPerSec;
  }

  // -------- Core periodic --------

  @Override
  public void rbsiPeriodic() {

    long epoch = drive.getPoseResetEpoch();
    if (epoch != lastSeenPoseResetEpoch) {
      lastSeenPoseResetEpoch = epoch;
      resetPoseGate(drive.getLastPoseResetTimestamp()); // your existing method
      Logger.recordOutput("Vision/PoseGateResetFromDrive", true);
    } else {
      Logger.recordOutput("Vision/PoseGateResetFromDrive", false);
    }

    // 1) Update/log camera inputs
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/Camera" + i, inputs[i]);
    }

    // 2) Pick one best accepted estimate per camera for this loop
    final ArrayList<TimedEstimate> perCamAccepted = new ArrayList<>(io.length);

    for (int cam = 0; cam < io.length; cam++) {
      int seen = 0;
      int accepted = 0;
      int rejected = 0;

      TimedEstimate best = null;
      double bestTrustScale = Double.NaN;
      int bestTrustedCount = 0;
      int bestTagCount = 0;

      for (var obs : inputs[cam].poseObservations) {
        seen++;

        GateResult gate = passesHardGatesAndYawGate(cam, obs);
        if (!gate.accepted) {
          rejected++;
          continue;
        }

        BuiltEstimate built = buildEstimate(cam, obs);
        if (built == null) {
          rejected++;
          continue;
        }

        TimedEstimate est = built.estimate;
        if (best == null || isBetter(est, best)) {
          best = est;
          bestTrustScale = built.trustScale;
          bestTrustedCount = built.trustedCount;
          bestTagCount = obs.tagCount();
        }
      }

      if (best != null) {
        accepted++;
        lastAcceptedTsPerCam[cam] = best.ts;
        perCamAccepted.add(best);

        Logger.recordOutput("Vision/Camera" + cam + "/InjectedPose2d", best.pose);
        Logger.recordOutput("Vision/Camera" + cam + "/InjectedTimestamp", best.ts);
        Logger.recordOutput("Vision/Camera" + cam + "/InjectedStdDevs", best.stdDevs);
        Logger.recordOutput("Vision/Camera" + cam + "/LastAcceptedTrustScale", bestTrustScale);
        Logger.recordOutput("Vision/Camera" + cam + "/LastAcceptedTrustedCount", bestTrustedCount);
        Logger.recordOutput("Vision/Camera" + cam + "/LastAcceptedTagCount", bestTagCount);
      }

      Logger.recordOutput("Vision/Camera" + cam + "/ObsSeen", seen);
      Logger.recordOutput("Vision/Camera" + cam + "/ObsAccepted", accepted);
      Logger.recordOutput("Vision/Camera" + cam + "/ObsRejected", rejected);
    }

    if (perCamAccepted.isEmpty()) return;

    // 3) Fusion time is the newest timestamp among accepted per-camera samples
    double tF = perCamAccepted.stream().mapToDouble(e -> e.ts).max().orElse(Double.NaN);
    if (!Double.isFinite(tF)) return;

    // 4) Time-align camera estimates to tF using odometry buffer, then inverse-variance fuse
    TimedEstimate fused = fuseAtTime(perCamAccepted, tF);
    if (fused == null) return;

    // 5) Smooth by fusing recent fused estimates (also aligned to tF)
    pushFused(fused);
    TimedEstimate smoothed = smoothAtTime(tF);
    if (smoothed == null) return;

    // 6) Inject
    consumer.accept(smoothed.pose, smoothed.ts, smoothed.stdDevs);

    Logger.recordOutput("Vision/FusedPose", fused.pose);
    Logger.recordOutput("Vision/SmoothedPose", smoothed.pose);
    Logger.recordOutput("Vision/FusedTimestamp", tF);
  }

  // -------- Gating + scoring --------

  private static final class GateResult {
    final boolean accepted;

    GateResult(boolean accepted) {
      this.accepted = accepted;
    }
  }

  private GateResult passesHardGatesAndYawGate(int cam, VisionIO.PoseObservation obs) {
    final double ts = obs.timestamp();

    // Monotonic per-camera time
    if (ts <= lastAcceptedTsPerCam[cam]) return new GateResult(false);

    // Reject anything older than last pose reset
    if (ts < lastPoseResetTimestamp) return new GateResult(false);

    // Must have tags
    if (obs.tagCount() <= 0) return new GateResult(false);

    // Single-tag ambiguity gate
    if (obs.tagCount() == 1 && obs.ambiguity() > maxAmbiguity) return new GateResult(false);

    // Z sanity
    if (Math.abs(obs.pose().getZ()) > maxZError) return new GateResult(false);

    // Field bounds
    Pose3d p = obs.pose();
    if (p.getX() < 0.0 || p.getX() > FieldConstants.aprilTagLayout.getFieldLength())
      return new GateResult(false);
    if (p.getY() < 0.0 || p.getY() > FieldConstants.aprilTagLayout.getFieldWidth())
      return new GateResult(false);

    // Optional 254-style yaw gate: only meaningful for single-tag
    if (enableSingleTagYawGate && obs.tagCount() == 1) {
      OptionalDouble maxYaw = drive.getMaxAbsYawRateRadPerSec(ts - yawGateLookbackSec, ts);
      if (maxYaw.isPresent() && maxYaw.getAsDouble() > yawGateLimitRadPerSec) {
        return new GateResult(false);
      }
    }

    return new GateResult(true);
  }

  private static final class BuiltEstimate {
    final TimedEstimate estimate;
    final double trustScale;
    final int trustedCount;

    BuiltEstimate(TimedEstimate estimate, double trustScale, int trustedCount) {
      this.estimate = estimate;
      this.trustScale = trustScale;
      this.trustedCount = trustedCount;
    }
  }

  private BuiltEstimate buildEstimate(int cam, VisionIO.PoseObservation obs) {
    // Base uncertainty grows with distance^2 / tagCount (your 2486-style)
    final double tagCount = Math.max(1, obs.tagCount());
    final double avgDist = Math.max(0.0, obs.averageTagDistance());
    final double distFactor = (avgDist * avgDist) / tagCount;

    final double camFactor = (cam < camConfigs.length) ? camConfigs[cam].stdDevFactor() : 1.0;

    double linearStdDev = linearStdDevBaseline * camFactor * distFactor;
    double angularStdDev = angularStdDevBaseline * camFactor * distFactor;

    // MegaTag2 bonus if applicable
    if (obs.type() == PoseObservationType.MEGATAG_2) {
      linearStdDev *= linearStdDevMegatag2Factor;
      angularStdDev *= angularStdDevMegatag2Factor;
    }

    // Trusted tag blending
    final Set<Integer> kTrusted = trustedTags.get();
    int trustedCount = 0;
    for (int id : obs.usedTagIds()) {
      if (kTrusted.contains(id)) trustedCount++;
    }

    if (requireTrustedTag && trustedCount == 0) {
      return null;
    }

    final int usedCount = obs.usedTagIds().size();
    final double fracTrusted = (usedCount == 0) ? 0.0 : ((double) trustedCount / usedCount);

    final double trustScale =
        untrustedTagStdDevScale + fracTrusted * (trustedTagStdDevScale - untrustedTagStdDevScale);

    linearStdDev *= trustScale;
    angularStdDev *= trustScale;

    // Output logs for tuning
    Logger.recordOutput("Vision/Camera" + cam + "/InjectedFracTrusted", fracTrusted);

    return new BuiltEstimate(
        new TimedEstimate(
            obs.pose().toPose2d(),
            obs.timestamp(),
            VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev)),
        trustScale,
        trustedCount);
  }

  private boolean isBetter(TimedEstimate a, TimedEstimate b) {
    // Lower trace of stddev vector (x+y+theta) is better
    double ta = a.stdDevs.get(0, 0) + a.stdDevs.get(1, 0) + a.stdDevs.get(2, 0);
    double tb = b.stdDevs.get(0, 0) + b.stdDevs.get(1, 0) + b.stdDevs.get(2, 0);
    return ta < tb;
  }

  // -------- Time alignment + fusion --------

  private TimedEstimate fuseAtTime(ArrayList<TimedEstimate> estimates, double tF) {
    final ArrayList<TimedEstimate> aligned = new ArrayList<>(estimates.size());
    for (var e : estimates) {
      Pose2d alignedPose = timeAlignPose(e.pose, e.ts, tF);
      if (alignedPose == null) return null;
      aligned.add(new TimedEstimate(alignedPose, tF, e.stdDevs));
    }
    return inverseVarianceFuse(aligned, tF);
  }

  private Pose2d timeAlignPose(Pose2d visionPoseAtTs, double ts, double tF) {
    Optional<Pose2d> odomAtTsOpt = drive.getPoseAtTime(ts);
    Optional<Pose2d> odomAtTFOpt = drive.getPoseAtTime(tF);
    if (odomAtTsOpt.isEmpty() || odomAtTFOpt.isEmpty()) return null;

    Pose2d odomAtTs = odomAtTsOpt.get();
    Pose2d odomAtTF = odomAtTFOpt.get();

    // Transform that takes odomAtTs -> odomAtTF
    Transform2d ts_T_tf = odomAtTF.minus(odomAtTs);

    // Apply same motion to vision pose to bring it forward
    return visionPoseAtTs.transformBy(ts_T_tf);
  }

  private TimedEstimate inverseVarianceFuse(ArrayList<TimedEstimate> alignedAtTF, double tF) {
    if (alignedAtTF.isEmpty()) return null;
    if (alignedAtTF.size() == 1) return alignedAtTF.get(0);

    double sumWx = 0, sumWy = 0, sumWth = 0;
    double sumX = 0, sumY = 0;
    double sumCos = 0, sumSin = 0;

    for (var e : alignedAtTF) {
      double sx = e.stdDevs.get(0, 0);
      double sy = e.stdDevs.get(1, 0);
      double sth = e.stdDevs.get(2, 0);

      double vx = sx * sx;
      double vy = sy * sy;
      double vth = sth * sth;

      double wx = 1.0 / vx;
      double wy = 1.0 / vy;
      double wth = 1.0 / vth;

      sumWx += wx;
      sumWy += wy;
      sumWth += wth;

      sumX += e.pose.getX() * wx;
      sumY += e.pose.getY() * wy;

      sumCos += e.pose.getRotation().getCos() * wth;
      sumSin += e.pose.getRotation().getSin() * wth;
    }

    Pose2d fused = new Pose2d(sumX / sumWx, sumY / sumWy, new Rotation2d(sumCos, sumSin));

    Matrix<N3, N1> fusedStd =
        VecBuilder.fill(Math.sqrt(1.0 / sumWx), Math.sqrt(1.0 / sumWy), Math.sqrt(1.0 / sumWth));

    return new TimedEstimate(fused, tF, fusedStd);
  }

  // -------- Smoothing buffer --------

  private void pushFused(TimedEstimate fused) {
    fusedBuffer.addLast(fused);

    while (fusedBuffer.size() > smoothMaxSize) {
      fusedBuffer.removeFirst();
    }

    // Trim by time window relative to newest
    while (!fusedBuffer.isEmpty() && fused.ts - fusedBuffer.peekFirst().ts > smoothWindowSec) {
      fusedBuffer.removeFirst();
    }
  }

  private TimedEstimate smoothAtTime(double tF) {
    if (fusedBuffer.isEmpty()) return null;
    if (fusedBuffer.size() == 1) return fusedBuffer.peekLast();

    final ArrayList<TimedEstimate> aligned = new ArrayList<>(fusedBuffer.size());
    for (var e : fusedBuffer) {
      Pose2d alignedPose = timeAlignPose(e.pose, e.ts, tF);
      if (alignedPose == null) continue;
      aligned.add(new TimedEstimate(alignedPose, tF, e.stdDevs));
    }

    if (aligned.isEmpty()) return fusedBuffer.peekLast();
    return inverseVarianceFuse(aligned, tF);
  }
}
