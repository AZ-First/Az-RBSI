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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants.Cameras;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import frc.robot.util.TimedPose;
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

  // Declare the Vision IO
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;

  /** Vision Consumer definition */
  @FunctionalInterface
  public interface PoseMeasurementConsumer {
    void accept(TimedPose measurement);
  }

  // Declare pose consumer, drivebase, and epoch reset
  private final PoseMeasurementConsumer consumer;
  private final Drive drive;
  private long lastSeenPoseResetEpoch = -1;

  // Declare the camera configurations
  private final Cameras.CameraConfig[] camConfigs = Cameras.ALL;

  // Per-camera monotonic and pose reset gates
  private final double[] lastAcceptedTsPerCam;
  private volatile double lastPoseResetTimestamp = Double.NEGATIVE_INFINITY;

  // Smoothing buffer (recent fused estimates)
  private final ArrayDeque<TimedPose> fusedBuffer = new ArrayDeque<>();
  private final double smoothWindowSec = 0.25;
  private final int smoothMaxSize = 12;

  // Trusted tags configuration (swappable per event/field)
  private final AtomicReference<Set<Integer>> trustedTags = new AtomicReference<>(Set.of());
  private volatile boolean requireTrustedTag = false;

  // Scale factors applied based on fraction of trusted tags in an observation
  private volatile double trustedTagStdDevScale = 0.70; // < 1 => more trusted
  private volatile double untrustedTagStdDevScale = 1.40; // > 1 => less trusted

  // Yaw-rate gate for single-tag measurements
  private volatile boolean enableSingleTagYawGate = true;
  private volatile double yawGateLookbackSec = 0.30;
  private volatile double yawGateLimitRadPerSec = 5.0;

  // Variance minimum for fusing poses to prevent divide-by-zero explosions
  private static final double kMinVariance = 1e-12;

  /** Constructor */
  public Vision(Drive drive, PoseMeasurementConsumer consumer, VisionIO... io) {
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

  /**
   * Priority value for this virtual subsystem
   *
   * <p>See `frc.robot.util.VirtualSubsystem` for a description of the suggested values for various
   * virtual subsystems.
   */
  @Override
  protected int getPeriodPriority() {
    return -10;
  }

  /** Periodic Function */
  @Override
  public void rbsiPeriodic() {

    // Pose reset logic and logging
    long epoch = drive.getPoseResetEpoch();
    if (epoch != lastSeenPoseResetEpoch) {
      lastSeenPoseResetEpoch = epoch;
      resetPoseGate(drive.getLastPoseResetTimestamp());
      Logger.recordOutput("Vision/PoseGateResetFromDrive", true);
    } else {
      Logger.recordOutput("Vision/PoseGateResetFromDrive", false);
    }

    // Update & log camera inputs
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/Camera" + i, inputs[i]);
    }

    // Pick the one best accepted estimate per camera for this loop
    final ArrayList<TimedPose> perCamAccepted = new ArrayList<>(io.length);

    // Loop over cameras
    for (int cam = 0; cam < io.length; cam++) {

      // Instantiate variables for this camera
      int seen = 0;
      int accepted = 0;
      int rejected = 0;
      TimedPose best = null;
      double bestTrustScale = Double.NaN;
      int bestTrustedCount = 0;
      int bestTagCount = 0;

      // Loop over observations for this camera this loop
      for (var obs : inputs[cam].poseObservations) {

        // Increment
        seen++;

        // Check the gating criteria; move on if bad
        GateResult gate = passesHardGatesAndYawGate(cam, obs);
        Logger.recordOutput("Vision/Camera" + cam + "/GateFail", gate.reason);
        if (!gate.accepted) {
          rejected++;
          continue;
        }

        // Build a pose estimate; move on if bad
        BuiltEstimate built = buildEstimate(cam, obs);
        if (built == null) {
          rejected++;
          continue;
        }

        // If this estimate is better than the existing "best", update this -> best
        TimedPose est = built.estimate;
        if (best == null || isBetter(est, best)) {
          best = est;
          bestTrustScale = built.trustScale;
          bestTrustedCount = built.trustedCount;
          bestTagCount = obs.tagCount();
        }
      }

      // Add an accepted measurement, and update
      if (best != null) {
        accepted++;
        lastAcceptedTsPerCam[cam] = best.timestampSeconds();
        perCamAccepted.add(best);

        Logger.recordOutput("Vision/Camera" + cam + "/InjectedPose2d", best.pose());
        Logger.recordOutput("Vision/Camera" + cam + "/InjectedTimestamp", best.timestampSeconds());
        Logger.recordOutput("Vision/Camera" + cam + "/InjectedStdDevs", best.stdDevs());
        Logger.recordOutput("Vision/Camera" + cam + "/LastAcceptedTrustScale", bestTrustScale);
        Logger.recordOutput("Vision/Camera" + cam + "/LastAcceptedTrustedCount", bestTrustedCount);
        Logger.recordOutput("Vision/Camera" + cam + "/LastAcceptedTagCount", bestTagCount);
      }

      // Log everything from this camera
      Logger.recordOutput("Vision/Camera" + cam + "/ObsSeen", seen);
      Logger.recordOutput("Vision/Camera" + cam + "/ObsAccepted", accepted);
      Logger.recordOutput("Vision/Camera" + cam + "/ObsRejected", rejected);
    }

    // If no "accepted" pose measurements from this loop, return now
    if (perCamAccepted.isEmpty()) return;

    // Fusion time is the newest timestamp among accepted per-camera samples; return if NaN
    double tFusion =
        perCamAccepted.stream().mapToDouble(e -> e.timestampSeconds()).max().orElse(Double.NaN);
    if (!Double.isFinite(tFusion)) return;

    // Time-align camera estimates to tFusion using odometry buffer, then inverse-variance fuse;
    // return if null
    TimedPose fused = fuseAtTime(perCamAccepted, tFusion);
    if (fused == null) return;

    // Smooth by fusing recent fused estimates (also aligned to tFusion); return if null
    // NOTE: THIS IS WHERE THE PROBLEM LIES, I THINK.  THE FUSED POSE IS OKAY, BUT THE SMOOTHED POSE
    // IS NOT!!!
    pushFused(fused);
    TimedPose smoothed = smoothAtTime(tFusion);
    if (smoothed == null) return;

    // Inject the pose
    consumer.accept(smoothed);

    // Log the fused and smoothed poses along with tFusion
    Logger.recordOutput("Vision/FusedPose", fused.pose());
    Logger.recordOutput("Vision/SmoothedPose", smoothed.pose());
    Logger.recordOutput("Vision/FusedTimestamp", tFusion);
  }

  /************************************************************************* */
  /** Runtime configuration hooks ****************************************** */

  /**
   * Call when odometry is reset (e.g. auto start, manual reset, etc).
   *
   * @param fpgaNowSeconds Timestamp for the pose gate reset
   */
  public void resetPoseGate(double fpgaNowSeconds) {
    lastPoseResetTimestamp = fpgaNowSeconds;
    fusedBuffer.clear();
    Arrays.fill(lastAcceptedTsPerCam, Double.NEGATIVE_INFINITY);
  }

  /**
   * Swap trusted tag set per event/field without redeploy
   *
   * @param tags Set of trusted tags to use
   */
  public void setTrustedTags(Set<Integer> tags) {
    trustedTags.set(Set.copyOf(tags));
  }

  /**
   * Set whether to require trusted tags
   *
   * @param require Boolean
   */
  public void setRequireTrustedTag(boolean require) {
    requireTrustedTag = require;
  }

  /**
   * Set the (un)trusted standard deviation scales
   *
   * @param trustedScale The scale for trusted tags
   * @param untrustedScale The scale for untrusted tags
   */
  public void setTrustedTagStdDevScales(double trustedScale, double untrustedScale) {
    trustedTagStdDevScale = trustedScale;
    untrustedTagStdDevScale = untrustedScale;
  }

  /**
   * Set the yaw gate for single-tag measurements
   *
   * @param enabled Enable the gate?
   * @param lookbackSec Lookback time
   * @param limitRadPerSec Yaw rate above which single-tag measurements will be ignored
   */
  public void setSingleTagYawGate(boolean enabled, double lookbackSec, double limitRadPerSec) {
    enableSingleTagYawGate = enabled;
    yawGateLookbackSec = lookbackSec;
    yawGateLimitRadPerSec = limitRadPerSec;
  }

  /************************************************************************* */
  /** Gating + Scoring ***************************************************** */

  /** GateResult Class */
  private static final class GateResult {
    final boolean accepted;
    final String reason;

    GateResult(boolean accepted, String reason) {
      this.accepted = accepted;
      this.reason = reason;
    }
  }

  /**
   * Gating Function -- checks all sorts of things!
   *
   * @param cam Camera index
   * @param obs PoseObservation
   */
  private GateResult passesHardGatesAndYawGate(int cam, VisionIO.PoseObservation obs) {
    final double ts = obs.timestamp();

    // Monotonic per-camera time
    if (ts <= lastAcceptedTsPerCam[cam]) return new GateResult(false, "monotonic time");

    // Reject anything older than last pose reset
    if (ts < lastPoseResetTimestamp) return new GateResult(false, "older than pose reset");

    // Must have tags
    if (obs.tagCount() <= 0) return new GateResult(false, "no tags");

    // Single-tag ambiguity gate
    if (obs.tagCount() == 1 && obs.ambiguity() > maxAmbiguity)
      return new GateResult(false, "highly ambiguous");

    // Z sanity
    if (Math.abs(obs.pose().getZ()) > maxZError) return new GateResult(false, "z not sane");

    // Field bounds
    Pose3d p = obs.pose();
    if (p.getX() < 0.0 || p.getX() > FieldConstants.aprilTagLayout.getFieldLength())
      return new GateResult(false, "out of bounds field X");
    if (p.getY() < 0.0 || p.getY() > FieldConstants.aprilTagLayout.getFieldWidth())
      return new GateResult(false, "out of bounds field Y");

    // Optional yaw gate: only meaningful for single-tag
    if (enableSingleTagYawGate && obs.tagCount() == 1) {
      OptionalDouble maxYaw = drive.getMaxAbsYawRateRadPerSec(ts - yawGateLookbackSec, ts);
      if (maxYaw.isPresent() && maxYaw.getAsDouble() > yawGateLimitRadPerSec) {
        return new GateResult(false, "YAW gate failed");
      }
    }

    return new GateResult(true, "");
  }

  /** Built Estimate Class */
  private static final class BuiltEstimate {
    final TimedPose estimate;
    final double trustScale;
    final int trustedCount;

    BuiltEstimate(TimedPose estimate, double trustScale, int trustedCount) {
      this.estimate = estimate;
      this.trustScale = trustScale;
      this.trustedCount = trustedCount;
    }
  }

  /**
   * Build a pose esitmate
   *
   * @param cam Camera Index
   * @param obs PoseObservation
   */
  private BuiltEstimate buildEstimate(int cam, VisionIO.PoseObservation obs) {
    // Base uncertainty grows with distance^2 / tagCount
    final double tagCount = Math.max(1, obs.tagCount());
    final double avgDist = Math.max(0.0, obs.averageTagDistance());
    final double distFactor = (avgDist * avgDist) / tagCount;

    // Camera uncertainty factor
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

    // If no trusted tags, return null
    if (requireTrustedTag && trustedCount == 0) {
      return null;
    }

    // Build the trust scale
    final int usedCount = obs.usedTagIds().size();
    final double fracTrusted = (usedCount == 0) ? 0.0 : ((double) trustedCount / usedCount);
    final double trustScale =
        untrustedTagStdDevScale + fracTrusted * (trustedTagStdDevScale - untrustedTagStdDevScale);

    linearStdDev *= trustScale;
    angularStdDev *= trustScale;

    // Output logs for tuning
    Logger.recordOutput("Vision/Camera" + cam + "/InjectedFracTrusted", fracTrusted);

    return new BuiltEstimate(
        new TimedPose(
            obs.pose().toPose2d(),
            obs.timestamp(),
            VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev)),
        trustScale,
        trustedCount);
  }

  /**
   * Evaluate whether a is better than b
   *
   * @param a Base pose
   * @param b Competitor pose
   */
  private boolean isBetter(TimedPose a, TimedPose b) {
    // Lower trace of stddev vector (x+y+theta) is better
    double ta = a.stdDevs().get(0, 0) + a.stdDevs().get(1, 0) + a.stdDevs().get(2, 0);
    double tb = b.stdDevs().get(0, 0) + b.stdDevs().get(1, 0) + b.stdDevs().get(2, 0);
    return ta < tb;
  }

  /************************************************************************* */
  /** Time alignment & fusion ********************************************** */

  /**
   * Fuse poses at a specified timestamp
   *
   * @param estimates Array of TimedPose esitmates
   * @param tFusion The timestamp at which to fuse the measurements
   */
  private TimedPose fuseAtTime(ArrayList<TimedPose> estimates, double tFusion) {
    final ArrayList<TimedPose> aligned = new ArrayList<>(estimates.size());
    for (var e : estimates) {
      Pose2d alignedPose = timeAlignPose(e.pose(), e.timestampSeconds(), tFusion);
      if (alignedPose == null) return null;
      aligned.add(new TimedPose(alignedPose, tFusion, e.stdDevs()));
    }
    return inverseVarianceFuse(aligned, tFusion);
  }

  /**
   * Align a pose to where it would have been at the fusion time
   *
   * <p>Gets the odometric poses at ts and tFusion from the drivebase PoseEstimator, computes the
   * transform between them, and applies that to the vision pose.
   *
   * @param visionPoseAtTs The pose at ts
   * @param ts Timestamp of the pose
   * @param tFusion Fusion timestamp
   * @return Transformed Pose2d
   */
  private Pose2d timeAlignPose(Pose2d visionPoseAtTs, double ts, double tFusion) {
    Optional<Pose2d> odomAtTsOpt = drive.getPoseAtTime(ts);
    Optional<Pose2d> odomAtTFOpt = drive.getPoseAtTime(tFusion);
    // If empty, return null
    if (odomAtTsOpt.isEmpty() || odomAtTFOpt.isEmpty()) return null;

    // Transform that takes odomAtTs -> odomAtTF
    Transform2d ts_T_tf = odomAtTFOpt.get().minus(odomAtTsOpt.get());

    // Apply same motion to vision pose to bring it forward
    return visionPoseAtTs.transformBy(ts_T_tf);
  }

  /**
   * Fuse a list of poses using inverse variable weighting
   *
   * @param alignedAtTF List of aligned poses
   * @param tFusion Fusion timestamp
   * @return Fuesed TimedPose object
   */
  private TimedPose inverseVarianceFuse(ArrayList<TimedPose> alignedAtTF, double tFusion) {
    // If size of alignedAtTF is 0 or 1, return null or the only value
    if (alignedAtTF == null || alignedAtTF.isEmpty()) return null;
    if (alignedAtTF.size() == 1) return alignedAtTF.get(0);

    // Define summing values
    double sumWx = 0.0, sumWy = 0.0, sumWth = 0.0;
    double sumX = 0.0, sumY = 0.0;
    double sumCos = 0.0, sumSin = 0.0;

    // Loop over poses in the list
    for (int i = 0; i < alignedAtTF.size(); i++) {
      final TimedPose e = alignedAtTF.get(i);
      final Pose2d p = e.pose();
      final Matrix<N3, N1> s = e.stdDevs();

      final double sx = s.get(0, 0);
      final double sy = s.get(1, 0);
      final double sth = s.get(2, 0);

      // variance = std^2, clamp away from 0
      final double vx = Math.max(sx * sx, kMinVariance);
      final double vy = Math.max(sy * sy, kMinVariance);
      final double vth = Math.max(sth * sth, kMinVariance);

      final double wx = 1.0 / vx;
      final double wy = 1.0 / vy;
      final double wth = 1.0 / vth;

      // If any weight goes NaN/Inf, skip this measurement rather than poisoning the fuse.
      if (!Double.isFinite(wx) || !Double.isFinite(wy) || !Double.isFinite(wth)) {
        continue;
      }

      sumWx += wx;
      sumWy += wy;
      sumWth += wth;

      sumX += p.getX() * wx;
      sumY += p.getY() * wy;

      final Rotation2d rot = p.getRotation();
      sumCos += rot.getCos() * wth;
      sumSin += rot.getSin() * wth;
    }

    // If everything got skipped; return null
    if (sumWx <= 0.0 || sumWy <= 0.0 || sumWth <= 0.0) return null;

    // Construct the fused translation
    final Translation2d fusedTranslation = new Translation2d(sumX / sumWx, sumY / sumWy);

    // Rotation2d(cos, sin) will normalize internally; if both are ~0, fall back to zero.
    final Rotation2d fusedRotation =
        (Math.abs(sumCos) < 1e-12 && Math.abs(sumSin) < 1e-12)
            ? Rotation2d.kZero
            : new Rotation2d(sumCos, sumSin);

    // The fused pose is the combination of translation and rotation
    final Pose2d fusedPose = new Pose2d(fusedTranslation, fusedRotation);

    // Fused standard deviations
    final Matrix<N3, N1> fusedStdDevs =
        VecBuilder.fill(Math.sqrt(1.0 / sumWx), Math.sqrt(1.0 / sumWy), Math.sqrt(1.0 / sumWth));

    // Construct and return the TimedPose objects
    return new TimedPose(fusedPose, tFusion, fusedStdDevs);
  }

  /************************************************************************* */
  /** Smoothing buffer ***************************************************** */

  /** THIS LIKELY HAS PROBLEMS */

  /** Push the fused pose */
  private void pushFused(TimedPose fused) {
    fusedBuffer.addLast(fused);

    while (fusedBuffer.size() > smoothMaxSize) {
      fusedBuffer.removeFirst();
    }

    // Trim by time window relative to newest
    while (!fusedBuffer.isEmpty()
        && fused.timestampSeconds() - fusedBuffer.peekFirst().timestampSeconds()
            > smoothWindowSec) {
      fusedBuffer.removeFirst();
    }
  }

  private TimedPose smoothAtTime(double tFusion) {
    if (fusedBuffer.isEmpty()) return null;
    if (fusedBuffer.size() == 1) return fusedBuffer.peekLast();

    final ArrayList<TimedPose> aligned = new ArrayList<>(fusedBuffer.size());
    for (var e : fusedBuffer) {
      Pose2d alignedPose = timeAlignPose(e.pose(), e.timestampSeconds(), tFusion);
      if (alignedPose == null) continue;
      aligned.add(new TimedPose(alignedPose, tFusion, e.stdDevs()));
    }

    if (aligned.isEmpty()) return fusedBuffer.peekLast();
    return inverseVarianceFuse(aligned, tFusion);
  }
}
