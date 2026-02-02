// Copyright (c) 2024-2026 Az-FIRST
// http://github.com/AZ-First
// Copyright (c) 2024-2025 FRC 2486
// http://github.com/Coconuts2486-FRC
// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the AdvantageKit-License.md file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import static frc.robot.Constants.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import frc.robot.util.VirtualSubsystem;
import java.util.ArrayList;
import org.littletonrobotics.junction.Logger;

public class Vision extends VirtualSubsystem {
  private final VisionConsumer consumer;
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;

  // Camera configs (names, transforms, stddev multipliers, sim props)
  private final Constants.Cameras.CameraConfig[] camConfigs;

  // ---------------- Reusable scratch buffers (avoid per-loop allocations) ----------------
  // Summary buffers
  private final ArrayList<Pose3d> allTagPoses = new ArrayList<>(32);
  private final ArrayList<Pose3d> allRobotPoses = new ArrayList<>(64);
  private final ArrayList<Pose3d> allRobotPosesAccepted = new ArrayList<>(64);
  private final ArrayList<Pose3d> allRobotPosesRejected = new ArrayList<>(64);

  // Per-camera buffers (reused each camera)
  private final ArrayList<Pose3d> tagPoses = new ArrayList<>(16);
  private final ArrayList<Pose3d> robotPoses = new ArrayList<>(32);
  private final ArrayList<Pose3d> robotPosesAccepted = new ArrayList<>(32);
  private final ArrayList<Pose3d> robotPosesRejected = new ArrayList<>(32);

  public Vision(VisionConsumer consumer, VisionIO... io) {
    this.consumer = consumer;
    this.io = io;

    this.camConfigs = Constants.Cameras.ALL;

    // Initialize inputs
    this.inputs = new VisionIOInputsAutoLogged[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }

    // Initialize disconnected alerts
    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < io.length; i++) {
      disconnectedAlerts[i] =
          new Alert("Vision camera " + i + " is disconnected.", AlertType.kWarning);
    }

    // Log robot-to-camera transforms from the new camera config array
    // (Only log as many as exist in BOTH configs and IOs)
    int n = Math.min(camConfigs.length, io.length);
    for (int i = 0; i < n; i++) {
      Logger.recordOutput("Vision/RobotToCamera" + i, camConfigs[i].robotToCamera());
    }
  }

  /** Returns the X angle to the best target, useful for simple servoing. */
  public Rotation2d getTargetX(int cameraIndex) {
    return inputs[cameraIndex].latestTargetObservation.tx();
  }

  @Override
  public void rbsiPeriodic() {
    // 1) Update inputs + process inputs first (keeps AK logs consistent)
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/Camera" + i, inputs[i]);
      disconnectedAlerts[i].set(!inputs[i].connected);
    }

    // 2) Clear summary buffers (reused)
    allTagPoses.clear();
    allRobotPoses.clear();
    allRobotPosesAccepted.clear();
    allRobotPosesRejected.clear();

    // 3) Process each camera
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      // Clear per-camera buffers
      tagPoses.clear();
      robotPoses.clear();
      robotPosesAccepted.clear();
      robotPosesRejected.clear();

      // Add tag poses from ids
      for (int tagId : inputs[cameraIndex].tagIds) {
        var tagPose = FieldConstants.aprilTagLayout.getTagPose(tagId);
        if (tagPose.isPresent()) {
          tagPoses.add(tagPose.get());
        }
      }

      // Loop over pose observations
      for (var observation : inputs[cameraIndex].poseObservations) {
        // Reject rules
        boolean rejectPose =
            observation.tagCount() == 0
                || (observation.tagCount() == 1 && observation.ambiguity() > maxAmbiguity)
                || Math.abs(observation.pose().getZ()) > maxZError
                || observation.pose().getX() < 0.0
                || observation.pose().getX() > FieldConstants.aprilTagLayout.getFieldLength()
                || observation.pose().getY() < 0.0
                || observation.pose().getY() > FieldConstants.aprilTagLayout.getFieldWidth();

        // Log pose buckets
        robotPoses.add(observation.pose());
        if (rejectPose) {
          robotPosesRejected.add(observation.pose());
        } else {
          robotPosesAccepted.add(observation.pose());
        }

        if (rejectPose) {
          continue;
        }

        // Standard deviations
        double stdDevFactor =
            Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
        double linearStdDev = linearStdDevBaseline * stdDevFactor;
        double angularStdDev = angularStdDevBaseline * stdDevFactor;

        if (observation.type() == PoseObservationType.MEGATAG_2) {
          linearStdDev *= linearStdDevMegatag2Factor;
          angularStdDev *= angularStdDevMegatag2Factor;
        }

        // Apply per-camera multiplier from CameraConfig
        if (cameraIndex < camConfigs.length) {
          double k = camConfigs[cameraIndex].stdDevFactor();
          linearStdDev *= k;
          angularStdDev *= k;
        }

        consumer.accept(
            observation.pose().toPose2d(),
            observation.timestamp(),
            VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
      }

      // Per-camera logs (arrays allocate; acceptable if you’re OK with this in the log loop)
      Logger.recordOutput(
          "Vision/Camera" + cameraIndex + "/TagPoses", tagPoses.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/Camera" + cameraIndex + "/RobotPoses", robotPoses.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/Camera" + cameraIndex + "/RobotPosesAccepted",
          robotPosesAccepted.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/Camera" + cameraIndex + "/RobotPosesRejected",
          robotPosesRejected.toArray(new Pose3d[0]));

      // Aggregate summary
      allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(robotPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
      allRobotPosesRejected.addAll(robotPosesRejected);
    }

    // 4) Summary logs
    Logger.recordOutput("Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[0]));
    Logger.recordOutput("Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[0]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesAccepted", allRobotPosesAccepted.toArray(new Pose3d[0]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesRejected", allRobotPosesRejected.toArray(new Pose3d[0]));
  }

  @FunctionalInterface
  public static interface VisionConsumer {
    void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }
}
