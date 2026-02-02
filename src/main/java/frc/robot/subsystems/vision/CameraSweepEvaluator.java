// Copyright (c) 2026 Az-FIRST
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

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import org.littletonrobotics.junction.Logger;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;
import org.photonvision.targeting.PhotonPipelineResult;

/** Sweeps two simulated cameras in a grid of poses and writes a CSV of performance. */
public class CameraSweepEvaluator {

  private final VisionSystemSim visionSim;
  private final PhotonCameraSim camSim1;
  private final PhotonCameraSim camSim2;

  /** Indexes into Constants.Cameras.ALL for each camera being swept. */
  private final int cam1Index;

  private final int cam2Index;

  public CameraSweepEvaluator(
      VisionSystemSim visionSim, PhotonCameraSim camSim1, PhotonCameraSim camSim2) {
    this(visionSim, camSim1, camSim2, 0, 1);
  }

  public CameraSweepEvaluator(
      VisionSystemSim visionSim,
      PhotonCameraSim camSim1,
      PhotonCameraSim camSim2,
      int cam1Index,
      int cam2Index) {
    this.visionSim = visionSim;
    this.camSim1 = camSim1;
    this.camSim2 = camSim2;
    this.cam1Index = cam1Index;
    this.cam2Index = cam2Index;
  }

  private static Quaternion quatFromAxisAngle(double ax, double ay, double az, double angleRad) {
    double half = 0.5 * angleRad;
    double s = Math.sin(half);
    double c = Math.cos(half);

    // normalize axis defensively
    double n = Math.sqrt(ax * ax + ay * ay + az * az);
    if (n < 1e-12) {
      return new Quaternion(1.0, 0.0, 0.0, 0.0);
    }
    ax /= n;
    ay /= n;
    az /= n;

    return new Quaternion(c, ax * s, ay * s, az * s);
  }

  // Hamilton product: q = a ⊗ b
  private static Quaternion quatMul(Quaternion a, Quaternion b) {
    double aw = a.getW(), ax = a.getX(), ay = a.getY(), az = a.getZ();
    double bw = b.getW(), bx = b.getX(), by = b.getY(), bz = b.getZ();

    double w = aw * bw - ax * bx - ay * by - az * bz;
    double x = aw * bx + ax * bw + ay * bz - az * by;
    double y = aw * by - ax * bz + ay * bw + az * bx;
    double z = aw * bz + ax * by - ay * bx + az * bw;

    // normalize to keep drift down
    double n = Math.sqrt(w * w + x * x + y * y + z * z);
    if (n < 1e-12) return new Quaternion(1.0, 0.0, 0.0, 0.0);
    return new Quaternion(w / n, x / n, y / n, z / n);
  }

  /**
   * Compose base rotation with an "extra" camera yaw/pitch.
   *
   * <p>Conventions: yawDeg = rotation about +Z (up), pitchDeg = rotation about +Y.
   *
   * <p>Order: extra = yaw ⊗ pitch (yaw first, then pitch, both in the camera/base frame) combined =
   * base ⊗ extra
   */
  private static Rotation3d composeCameraExtra(Rotation3d base, double yawDeg, double pitchDeg) {
    double yaw = Units.degreesToRadians(yawDeg);
    double pitch = Units.degreesToRadians(pitchDeg);

    Quaternion qBase = base.getQuaternion();

    Quaternion qYaw = quatFromAxisAngle(0.0, 0.0, 1.0, yaw);
    Quaternion qPitch = quatFromAxisAngle(0.0, 1.0, 0.0, pitch);

    Quaternion qExtra = quatMul(qYaw, qPitch);
    Quaternion qCombined = quatMul(qBase, qExtra);

    return new Rotation3d(qCombined);
  }

  private Transform3d getMountTransformOrThrow(int idx) {
    var all = Constants.Cameras.ALL;
    if (idx < 0 || idx >= all.length) {
      throw new IllegalArgumentException(
          "CameraSweepEvaluator camera index out of range: "
              + idx
              + " (ALL.length="
              + all.length
              + ")");
    }
    return all[idx].robotToCamera();
  }

  /** Run a full sweep of candidate camera placements and write a CSV of results. */
  public void runFullSweep(String outputCsvPath) throws IOException {
    // Example field bounds (meters) -- tune these for your field size
    double[] fieldX = {
      0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0, 5.5, 6.0, 6.5, 7.0, 7.5, 8.0
    };
    double[] fieldY = {0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0, 5.5, 6.0, 6.5, 7.0, 7.5};
    double[] robotZ = {0.0};

    // Sweep robot yaw at each location (degrees)
    double[] robotYawDeg = {-180, -135, -90, -45, 0, 45, 90, 135};

    // Camera “extra” rotation sweep (applied on top of the mount rotation)
    double[] camYawDeg = {-15, 0, 15};
    double[] camPitchDeg = {-10, 0, 10};

    // Pull fixed mount transforms from Constants (new regime)
    final Transform3d rToC0 = getMountTransformOrThrow(cam1Index);
    final Transform3d rToC1 = getMountTransformOrThrow(cam2Index);

    // Extract mount transform translation + quaternion once for CSV (self-contained rows)
    final Translation3d c0T = rToC0.getTranslation();
    final Translation3d c1T = rToC1.getTranslation();
    final Quaternion c0Q = rToC0.getRotation().getQuaternion();
    final Quaternion c1Q = rToC1.getRotation().getQuaternion();

    // Build the target list ONCE (huge savings vs doing it inside the deepest loop)
    final Set<VisionTargetSim> simTargets = visionSim.getVisionTargets();
    final List<VisionTargetSim> targetList = new ArrayList<>(simTargets);

    try (FileWriter writer = new FileWriter(outputCsvPath)) {
      writer.write(
          "robotX,robotY,robotZ,robotYawDeg,"
              + "cam1YawDeg,cam1PitchDeg,cam2YawDeg,cam2PitchDeg,score,"
              + "mount0_idx,mount0_tx,mount0_ty,mount0_tz,mount0_qw,mount0_qx,mount0_qy,mount0_qz,"
              + "mount1_idx,mount1_tx,mount1_ty,mount1_tz,mount1_qw,mount1_qx,mount1_qy,mount1_qz\n");

      for (double rx : fieldX) {
        for (double ry : fieldY) {
          for (double rz : robotZ) {
            for (double rYaw : robotYawDeg) {
              Pose3d robotPose =
                  new Pose3d(
                      new Translation3d(rx, ry, rz),
                      new Rotation3d(0.0, 0.0, Units.degreesToRadians(rYaw)));

              // Base camera poses from mount transforms (rotate with robot yaw)
              Pose3d cam1BasePose = robotPose.transformBy(rToC0);
              Pose3d cam2BasePose = robotPose.transformBy(rToC1);

              for (double c1Yaw : camYawDeg) {
                for (double c1Pitch : camPitchDeg) {
                  Rotation3d cam1Rot =
                      composeCameraExtra(cam1BasePose.getRotation(), c1Yaw, c1Pitch);
                  Pose3d cam1Pose = new Pose3d(cam1BasePose.getTranslation(), cam1Rot);

                  for (double c2Yaw : camYawDeg) {
                    for (double c2Pitch : camPitchDeg) {
                      Rotation3d cam2Rot =
                          composeCameraExtra(cam2BasePose.getRotation(), c2Yaw, c2Pitch);
                      Pose3d cam2Pose = new Pose3d(cam2BasePose.getTranslation(), cam2Rot);

                      // Simulate camera processing (timestampSeconds can be 0 for this offline
                      // sweep)
                      PhotonPipelineResult res1 = camSim1.process(0, cam1Pose, targetList);
                      PhotonPipelineResult res2 = camSim2.process(0, cam2Pose, targetList);

                      // Score: count visible targets + bonus for multi-tag
                      int n1 = res1.getTargets().size();
                      int n2 = res2.getTargets().size();
                      double score = n1 + n2;
                      if (n1 >= 2) score += 2.0;
                      if (n2 >= 2) score += 2.0;

                      for (var t : res1.getTargets()) score -= t.getPoseAmbiguity() * 2.0;
                      for (var t : res2.getTargets()) score -= t.getPoseAmbiguity() * 2.0;

                      writer.write(
                          String.format(
                              "%.2f,%.2f,%.2f,%.1f,%.1f,%.1f,%.1f,%.1f,%.3f,"
                                  + "%d,%.6f,%.6f,%.6f,%.16g,%.16g,%.16g,%.16g,"
                                  + "%d,%.6f,%.6f,%.6f,%.16g,%.16g,%.16g,%.16g\n",
                              rx,
                              ry,
                              rz,
                              rYaw,
                              c1Yaw,
                              c1Pitch,
                              c2Yaw,
                              c2Pitch,
                              score,
                              cam1Index,
                              c0T.getX(),
                              c0T.getY(),
                              c0T.getZ(),
                              c0Q.getW(),
                              c0Q.getX(),
                              c0Q.getY(),
                              c0Q.getZ(),
                              cam2Index,
                              c1T.getX(),
                              c1T.getY(),
                              c1T.getZ(),
                              c1Q.getW(),
                              c1Q.getX(),
                              c1Q.getY(),
                              c1Q.getZ()));

                      // Optional logging: keep it light (or decimate) so it doesn’t dominate
                      // runtime
                      Logger.recordOutput("CameraSweep/Score", score);
                    }
                  }
                }
              }
            }
          }
        }
      }

      writer.flush();
    }
  }
}
