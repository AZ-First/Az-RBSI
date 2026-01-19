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

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import java.util.List;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;
import org.photonvision.targeting.PhotonPipelineResult;

public class CameraPlacementEvaluator {

  private final Drive driveSim;
  private final VisionSystemSim visionSim;

  public CameraPlacementEvaluator(Drive driveSim, VisionSystemSim visionSim) {
    this.driveSim = driveSim;
    this.visionSim = visionSim;
  }

  public double evaluateCameraPose(
      PhotonCameraSim simCam, Transform3d robotToCamera, SimCameraProperties props) {

    // Add or adjust camera transform in vision sim
    visionSim.adjustCamera(simCam, robotToCamera);

    // Test field poses
    Pose2d[] testPoses =
        new Pose2d[] {
          new Pose2d(1.0, 1.0, new Rotation2d(0)),
          new Pose2d(3.0, 1.0, new Rotation2d(Math.PI / 2)),
          new Pose2d(5.0, 3.0, new Rotation2d(Math.PI)),
          new Pose2d(2.0, 4.0, new Rotation2d(-Math.PI / 2)),
        };

    double totalScore = 0.0;
    double latencyMillis = 0.0;

    for (Pose2d pose : testPoses) {

      driveSim.resetPose(pose);

      for (int i = 0; i < 5; i++) {
        driveSim.simulationPeriodic();
        visionSim.update(driveSim.getPose());
        Timer.delay(Constants.loopPeriodSecs);
      }

      var maybeCamPose3d = visionSim.getCameraPose(simCam);
      if (maybeCamPose3d.isEmpty()) {
        continue;
      }
      Pose3d camFieldPose = maybeCamPose3d.get();

      List<VisionTargetSim> allTargets = (List<VisionTargetSim>) visionSim.getVisionTargets();

      PhotonPipelineResult result = simCam.process(latencyMillis, camFieldPose, allTargets);

      int tagsSeen = result.targets.size();

      Pose2d estimatedPose = driveSim.getPose();
      double poseError = pose.getTranslation().getDistance(estimatedPose.getTranslation());

      double score = tagsSeen - poseError;
      totalScore += score;
    }

    return totalScore / testPoses.length;
  }
}
