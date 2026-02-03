// Copyright (c) 2024-2026 Az-FIRST
// http://github.com/AZ-First
// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the AdvantageKit-License.md file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import static frc.robot.FieldConstants.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.Set;
import org.photonvision.PhotonCamera;

/** IO implementation for real PhotonVision hardware (pose already solved by PV). */
public class VisionIOPhotonVision implements VisionIO {
  protected final PhotonCamera camera;
  protected final Transform3d robotToCamera;

  /**
   * Creates a new VisionIOPhotonVision.
   *
   * @param name The configured name of the camera.
   * @param robotToCamera The 3D position of the camera relative to the robot.
   */
  public VisionIOPhotonVision(String name, Transform3d robotToCamera) {
    this.camera = new PhotonCamera(name);
    this.robotToCamera = robotToCamera;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.connected = camera.isConnected();

    // Cap the number of unread results processed per loop
    final int kMaxUnread = 5;

    final Set<Integer> unionTagIds = new HashSet<>();
    final ArrayList<PoseObservation> poseObservations = new ArrayList<>(kMaxUnread);

    double newestTargetTs = Double.NEGATIVE_INFINITY;
    Rotation2d bestYaw = Rotation2d.kZero;
    Rotation2d bestPitch = Rotation2d.kZero;

    int processed = 0;
    for (var result : camera.getAllUnreadResults()) {
      // Hard cap
      if (processed++ >= kMaxUnread) break;

      final double ts = result.getTimestampSeconds();

      if (result.hasTargets() && ts >= newestTargetTs) {
        newestTargetTs = ts;
        bestYaw = Rotation2d.fromDegrees(result.getBestTarget().getYaw());
        bestPitch = Rotation2d.fromDegrees(result.getBestTarget().getPitch());
      }

      if (result.multitagResult.isPresent()) {
        var multitag = result.multitagResult.get();

        Transform3d fieldToCamera = multitag.estimatedPose.best;
        Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
        Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

        // Calculate average tag distance
        double totalTagDistance = 0.0;
        for (var target : result.targets) {
          totalTagDistance += target.bestCameraToTarget.getTranslation().getNorm();
        }
        double avgTagDistance =
            result.targets.isEmpty() ? 0.0 : (totalTagDistance / result.targets.size());

        Set<Integer> used = new HashSet<>();
        for (int id : multitag.fiducialIDsUsed) {
          used.add(id);
          unionTagIds.add(id);
        }

        poseObservations.add(
            new PoseObservation(
                ts,
                robotPose,
                multitag.estimatedPose.ambiguity,
                multitag.fiducialIDsUsed.size(),
                avgTagDistance,
                PoseObservationType.PHOTONVISION,
                Set.copyOf(used)));

      } else if (!result.targets.isEmpty()) {
        var target = result.targets.get(0);

        var tagPose = aprilTagLayout.getTagPose(target.fiducialId);
        if (tagPose.isEmpty()) continue;

        Transform3d fieldToTarget =
            new Transform3d(tagPose.get().getTranslation(), tagPose.get().getRotation());
        Transform3d cameraToTarget = target.bestCameraToTarget;

        Transform3d fieldToCamera = fieldToTarget.plus(cameraToTarget.inverse());
        Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
        Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

        unionTagIds.add(target.fiducialId);

        poseObservations.add(
            new PoseObservation(
                ts,
                robotPose,
                target.poseAmbiguity,
                1,
                cameraToTarget.getTranslation().getNorm(),
                PoseObservationType.PHOTONVISION,
                Set.of(target.fiducialId)));
      }
    }

    inputs.latestTargetObservation =
        (newestTargetTs > Double.NEGATIVE_INFINITY)
            ? new TargetObservation(bestYaw, bestPitch)
            : new TargetObservation(Rotation2d.kZero, Rotation2d.kZero);

    inputs.poseObservations = poseObservations.toArray(new PoseObservation[0]);

    inputs.tagIds = new int[unionTagIds.size()];
    int i = 0;
    for (int id : unionTagIds) inputs.tagIds[i++] = id;
  }
}
