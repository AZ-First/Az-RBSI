// Copyright (c) 2024-2026 Az-FIRST
// http://github.com/AZ-First
// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the AdvantageKit-License.md file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.FieldConstants;
import java.util.function.Supplier;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

/** IO implementation for physics sim using PhotonVision simulator. */
public class VisionIOPhotonVisionSim extends VisionIOPhotonVision {
  private static VisionSystemSim visionSim;

  private final Supplier<Pose2d> poseSupplier;

  @SuppressWarnings("unused")
  private final PhotonCameraSim cameraSim;

  /**
   * Creates a new VisionIOPhotonVisionSim.
   *
   * @param name The name of the camera (PhotonVision camera name).
   * @param robotToCamera Camera pose relative to robot frame.
   * @param poseSupplier Supplier for the robot pose (field->robot) to use in simulation.
   */
  public VisionIOPhotonVisionSim(
      String name, Transform3d robotToCamera, Supplier<Pose2d> poseSupplier) {
    super(name, robotToCamera);
    this.poseSupplier = poseSupplier;

    // Initialize VisionSystemSim once for all cameras
    if (visionSim == null) {
      visionSim = new VisionSystemSim("main");
      visionSim.addAprilTags(FieldConstants.aprilTagLayout);
    }

    // Camera properties:
    // - If you have per-camera SimCameraProperties in Constants, pass them here instead.
    // - Otherwise keep the default and tune later.
    var cameraProperties = new SimCameraProperties();

    // Recommended defaults (feel free to tune)
    // cameraProperties.setCalibration(1280, 800, Rotation2d.fromDegrees(100));
    // cameraProperties.setFPS(20);
    // cameraProperties.setAvgLatencyMs(35);
    // cameraProperties.setLatencyStdDevMs(5);

    cameraSim = new PhotonCameraSim(camera, cameraProperties);
    visionSim.addCamera(cameraSim, robotToCamera);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    // NOTE: This updates the sim world every time a sim camera is polled.
    // That's fine (fast enough), but if you want "update once per loop," see note below.
    visionSim.update(poseSupplier.get());

    // Then pull results like normal (and emit PoseObservation + usedTagIds sets)
    super.updateInputs(inputs);
  }
}
