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
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.FieldConstants;
import java.util.function.Supplier;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

/** IO implementation for physics sim using PhotonVision simulator. */
public class VisionIOPhotonVisionSim extends VisionIOPhotonVision {
  private static VisionSystemSim visionSim;
  private static long lastUpdateUs = Long.MIN_VALUE;
  private static final long MIN_UPDATE_PERIOD_US = 1_000;

  private final Supplier<Pose2d> poseSupplier;

  /**
   * Creates a new VisionIOPhotonVisionSim.
   *
   * @param name The name of the camera (PhotonVision camera name).
   * @param robotToCamera Camera pose relative to robot frame.
   * @param poseSupplier Supplier for the robot pose (field->robot) to use in simulation.
   */
  public VisionIOPhotonVisionSim(
      String name, Transform3d robotToCamera, Supplier<Pose2d> poseSupplier) {
    this(name, robotToCamera, new SimCameraProperties(), poseSupplier);
  }

  /**
   * Creates a new VisionIOPhotonVisionSim.
   *
   * @param name The name of the camera (PhotonVision camera name).
   * @param robotToCamera Camera pose relative to robot frame.
   * @param cameraProperties Camera simulation calibration, latency, noise, and FPS.
   * @param poseSupplier Supplier for the robot pose (field->robot) to use in simulation.
   */
  public VisionIOPhotonVisionSim(
      String name,
      Transform3d robotToCamera,
      SimCameraProperties cameraProperties,
      Supplier<Pose2d> poseSupplier) {
    super(name, robotToCamera);
    this.poseSupplier = poseSupplier;

    // Initialize VisionSystemSim once for all cameras
    if (visionSim == null) {
      visionSim = new VisionSystemSim("main");
      visionSim.addAprilTags(FieldConstants.aprilTagLayout);
    }

    PhotonCameraSim cameraSim = new PhotonCameraSim(camera, cameraProperties);
    visionSim.addCamera(cameraSim, robotToCamera);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    updateVisionSimOncePerLoop();

    // Then pull results like normal (and emit PoseObservation + usedTagIds sets)
    super.updateInputs(inputs);
  }

  private void updateVisionSimOncePerLoop() {
    long nowUs = RobotController.getFPGATime();
    if (nowUs - lastUpdateUs < MIN_UPDATE_PERIOD_US) return;

    visionSim.update(poseSupplier.get());
    lastUpdateUs = nowUs;
  }
}
