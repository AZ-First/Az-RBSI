// Copyright (c) 2024 FRC 6995
// https://github.com/frc6995
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

package frc.robot.subsystems.vision.FRC6995;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.TriConsumer;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

public class Vision {
  public interface VisionConsumer extends TriConsumer<Pose2d, Double, Vector<N3>> {}

  private class Camera {
    String name;
    PhotonCamera camera;
    PhotonPoseEstimator estimator;
    PhotonCameraSim simCamera;
    Pose3d estimatedPose = new Pose3d();
    StructPublisher<Pose3d> rawPosePublisher;

    public Camera(
        String name,
        PhotonCamera camera,
        PhotonPoseEstimator estimator,
        PhotonCameraSim simCamera) {
      this.name = name;
      this.camera = camera;
      this.estimator = estimator;
      this.simCamera = simCamera;
      var table = NetworkTableInstance.getDefault().getTable("Cameras").getSubTable(name);
      rawPosePublisher = table.getStructTopic("rawPose", Pose3d.struct).publish();
    }

    public void setRawPose(Pose3d pose) {
      estimatedPose = pose;
      rawPosePublisher.accept(estimatedPose);
    }
  }
  ;

  public class VisionConstants {
    private static SimCameraProperties cameraProp = new SimCameraProperties();

    static {
      // A 640 x 480 camera with a 100 degree diagonal FOV.
      cameraProp.setCalibration(1280, 900, Rotation2d.fromDegrees(100));
      // Approximate detection noise with average and standard deviation error in pixels.
      cameraProp.setCalibError(0.12, 0.04);
      // Set the camera image capture framerate (Note: this is limited by robot loop rate).
      cameraProp.setFPS(20);
      // The average and standard deviation in milliseconds of image data latency.
      cameraProp.setAvgLatencyMs(35);
      cameraProp.setLatencyStdDevMs(5);
    }

    public static final SimCameraProperties SIM_CAMERA_PROPERTIES = cameraProp;

    public static final Map<String, Transform3d> CAMERAS =
        Map.of(
            "OV9281-BL",
            new Transform3d(
                -(Units.inchesToMeters(14.25) - 0.102),
                (Units.inchesToMeters(14.25) - 0.066),
                Units.inchesToMeters(8.5),
                new Rotation3d(
                    Units.degreesToRadians(0),
                    Units.degreesToRadians(-13.65),
                    Units.degreesToRadians(-170))),
            "OV9281-BR",
            new Transform3d(
                -(Units.inchesToMeters(14.25) - 0.102),
                -(Units.inchesToMeters(14.25) - 0.072),
                Units.inchesToMeters(8.5),
                new Rotation3d(
                    Units.degreesToRadians(0),
                    Units.degreesToRadians(-13.65),
                    Units.degreesToRadians(170))),
            "OV9281-FR",
            new Transform3d(
                Units.inchesToMeters(14.25) - 0.102,
                -(Units.inchesToMeters(14.25) - 0.112),
                Units.inchesToMeters(8.5),
                new Rotation3d(
                    Units.degreesToRadians(0),
                    Units.degreesToRadians(-13.65),
                    Units.degreesToRadians(10))),
            "OV9281-FL",
            new Transform3d(
                (Units.inchesToMeters(14.25) - 0.102),
                (Units.inchesToMeters(14.25) - 0.066),
                Units.inchesToMeters(8.5),
                new Rotation3d(
                    Units.degreesToRadians(0),
                    Units.degreesToRadians(-13.65),
                    Units.degreesToRadians(-10))));
    public static final AprilTagFieldLayout FIELD_LAYOUT =
        AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
  }

  VisionSystemSim visionSim = new VisionSystemSim("main");
  private List<Camera> m_cameras;
  private List<PhotonCamera> m_actualCameras;
  private List<PhotonCameraSim> m_simCameras;
  private VisionConsumer addVisionMeasurement;
  private Supplier<Pose2d> getPose;
  private double lastPoseResetTimestamp = 0;

  public void resetPose() {
    lastPoseResetTimestamp = Timer.getFPGATimestamp();
  }

  public Vision(VisionConsumer addVisionMeasurement, Supplier<Pose2d> getPose) {
    if (RobotBase.isSimulation()) {
      visionSim.addAprilTags(AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField));
    }

    this.addVisionMeasurement = addVisionMeasurement;
    this.getPose = getPose;
    m_cameras = new ArrayList<>();
    VisionConstants.CAMERAS
        .entrySet()
        .iterator()
        .forEachRemaining(
            (entry) -> {
              var translation = entry.getValue();
              var cam = new PhotonCamera(entry.getKey());
              var estimator =
                  new PhotonPoseEstimator(
                      VisionConstants.FIELD_LAYOUT,
                      PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                      translation);
              estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
              PhotonCameraSim simCam = null;
              if (RobotBase.isSimulation()) {
                simCam = new PhotonCameraSim(cam, VisionConstants.SIM_CAMERA_PROPERTIES);
                visionSim.addCamera(simCam, translation);
              }
              m_cameras.add(new Camera(entry.getKey(), cam, estimator, simCam));
            });
  }

  public boolean filterPose(Pose3d robotPose) {
    if (Math.abs(robotPose.getZ()) > 0.25) {
      return false;
    }
    return true;
  }

  private boolean isReef(int id) {
    return (id >= 6 && id <= 11) || (id >= 17 && id <= 22);
  }

  private void handleResult(Camera camera, PhotonPipelineResult result) {
    var estimator = camera.estimator;
    estimator.setReferencePose(getPose.get());

    var robotPoseOpt = estimator.update(result);
    if (robotPoseOpt.isEmpty()) {
      return;
    }
    camera.setRawPose(robotPoseOpt.get().estimatedPose);
    var pose = robotPoseOpt.get();
    if (pose.timestampSeconds < lastPoseResetTimestamp) {
      return;
    }
    if (!filterPose(pose.estimatedPose)) {
      return;
    }
    double xConfidence;
    double yConfidence;
    double angleConfidence;
    if (pose.targetsUsed.size() == 0) {
      return; // should never happen but measurement shouldn't be trusted
    }
    double closestDistance = 1000;
    double avgDistance = 0;
    double closeEnoughTgts = 0;
    boolean ignore = false;

    final double border = 15;
    for (var tgt : pose.targetsUsed) {

      // rule out
      for (var corner : tgt.detectedCorners) {
        if (MathUtil.isNear(0, corner.x, border)
            || MathUtil.isNear(
                VisionConstants.SIM_CAMERA_PROPERTIES.getResWidth(), corner.x, border)
            || MathUtil.isNear(0, corner.y, border)
            || MathUtil.isNear(
                VisionConstants.SIM_CAMERA_PROPERTIES.getResHeight(), corner.y, border)) {
          return;
        }
      }
      double tdist = tgt.getBestCameraToTarget().getTranslation().getNorm();
      if (pose.targetsUsed.size() < 2) {
        var trustedDistance =
            isReef(pose.targetsUsed.get(0).fiducialId)
                ? Units.feetToMeters(6)
                : Units.feetToMeters(6);
        if (tdist > trustedDistance) {
          return;

        } else {
          closeEnoughTgts = 1;
        }
      }
      avgDistance += tdist;
      if (tdist < closestDistance) {
        closestDistance = tdist;
      }
      if (pose.targetsUsed.size() >= 2) {
        var trustedDistance =
            isReef(pose.targetsUsed.get(0).fiducialId)
                ? Units.feetToMeters(10)
                : Units.feetToMeters(8);

        if (tdist <= trustedDistance) {
          closeEnoughTgts++;
        }
      }
      // ignore |= (tgt.getFiducialId() == 13);
      // ignore |= (tgt.getFiducialId() == 14);
    }
    if (ignore) {
      return;
    }
    double distance = avgDistance / pose.targetsUsed.size();
    // SmartDashboard.putNumber(camera.name + "/distance", distance);
    if (closeEnoughTgts == 0) {
      return;
    }
    if (pose.targetsUsed.size() < 2) {
      xConfidence = 0.5 * distance / 4.0;
      yConfidence = 0.5 * distance / 4.0;
      angleConfidence = 1;
    } else {
      xConfidence = 0.02 * distance * distance;
      yConfidence = 0.02 * distance * distance;
      angleConfidence = 0.3 * distance * distance;
    }
    this.addVisionMeasurement.accept(
        pose.estimatedPose.toPose2d(),
        Utils.fpgaToCurrentTime(pose.timestampSeconds),
        VecBuilder.fill(xConfidence, yConfidence, angleConfidence));
  }

  public void update() {
    visionSim.update(getPose.get());
    for (Camera camera : m_cameras) {
      for (PhotonPipelineResult result : camera.camera.getAllUnreadResults()) {
        handleResult(camera, result);
      }
    }
  }
}
