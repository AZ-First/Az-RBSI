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
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
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

  public CameraSweepEvaluator(
      VisionSystemSim visionSim, PhotonCameraSim camSim1, PhotonCameraSim camSim2) {
    this.visionSim = visionSim;
    this.camSim1 = camSim1;
    this.camSim2 = camSim2;
  }

  /**
   * Run a full sweep of candidate camera placements.
   *
   * @param outputCsvPath Path to write the CSV results
   */
  public void runFullSweep(String outputCsvPath) throws IOException {
    // Example field bounds (meters) -- tune these for your field size
    double[] fieldX = {0.0, 2.0, 4.0, 6.0, 8.0}; // 0–8m along field length
    double[] fieldY = {0.0, 1.0, 2.0, 3.0, 4.0}; // 0–4m along field width
    double[] robotZ = {0.0}; // Usually floor height
    double[] camYaw = {-30, 0, 30}; // Camera yaw
    double[] camPitch = {-10, 0, 10}; // Camera pitch

    try (FileWriter writer = new FileWriter(outputCsvPath)) {
      writer.write("robotX,robotY,robotZ," + "cam1Yaw,cam1Pitch,cam2Yaw,cam2Pitch,score\n");

      // Sweep robot over the field
      for (double rx : fieldX) {
        for (double ry : fieldY) {
          for (double rz : robotZ) {
            Pose3d robotPose =
                new Pose3d(
                    new Translation3d(rx, ry, rz),
                    new Rotation3d(0, 0, 0) // robot heading = 0 for simplicity
                    );

            // Sweep camera rotations
            for (double c1Yaw : camYaw) {
              for (double c1Pitch : camPitch) {
                Pose3d cam1Pose =
                    new Pose3d(
                        robotPose.getTranslation(),
                        new Rotation3d(Math.toRadians(c1Pitch), 0, Math.toRadians(c1Yaw)));

                for (double c2Yaw : camYaw) {
                  for (double c2Pitch : camPitch) {
                    Pose3d cam2Pose =
                        new Pose3d(
                            robotPose.getTranslation(),
                            new Rotation3d(Math.toRadians(c2Pitch), 0, Math.toRadians(c2Yaw)));

                    // Get all vision targets
                    Set<VisionTargetSim> simTargets = visionSim.getVisionTargets();
                    List<VisionTargetSim> targetList = new ArrayList<>(simTargets);

                    // Simulate camera processing
                    PhotonPipelineResult res1 = camSim1.process(0, cam1Pose, targetList);
                    PhotonPipelineResult res2 = camSim2.process(0, cam2Pose, targetList);

                    // Score
                    double score = res1.getTargets().size() + res2.getTargets().size();
                    if (res1.getTargets().size() >= 2) score += 2.0;
                    if (res2.getTargets().size() >= 2) score += 2.0;

                    // Penalize ambiguity safely with loops
                    for (var t : res1.getTargets()) score -= t.getPoseAmbiguity() * 2.0;
                    for (var t : res2.getTargets()) score -= t.getPoseAmbiguity() * 2.0;

                    // Write CSV row
                    writer.write(
                        String.format(
                            "%.2f,%.2f,%.2f,%.1f,%.1f,%.1f,%.1f,%.1f\n",
                            rx, ry, rz, c1Yaw, c1Pitch, c2Yaw, c2Pitch, score));

                    Logger.recordOutput("CameraSweep/Score", score);
                    Logger.recordOutput("CameraSweep/Cam1Pose", cam1Pose);
                    Logger.recordOutput("CameraSweep/Cam2Pose", cam2Pose);
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
