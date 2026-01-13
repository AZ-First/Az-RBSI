// Copyright (c) 2024-2026 Az-FIRST
// http://github.com/AZ-First
// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
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

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.IOException;
import java.nio.file.Path;
import lombok.Getter;

/**
 * Contains various field dimensions and useful reference points. All units are in meters and poses
 * have a blue alliance origin.
 */
public class FieldConstants {

  /** AprilTag Field Layout ************************************************ */
  public static final double aprilTagWidth = Inches.of(6.50).in(Meters);

  public static final String aprilTagFamily = "36h11";

  public static final double fieldLength = AprilTagLayoutType.OFFICIAL.getLayout().getFieldLength();
  public static final double fieldWidth = AprilTagLayoutType.OFFICIAL.getLayout().getFieldWidth();

  public static final int aprilTagCount = AprilTagLayoutType.OFFICIAL.getLayout().getTags().size();
  public static final AprilTagLayoutType defaultAprilTagType = AprilTagLayoutType.OFFICIAL;

  public static final AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  @Getter
  public enum AprilTagLayoutType {
    OFFICIAL("2026-official"),
    NONE("2026-none"),
    REEFSCAPE("2025-official");

    AprilTagLayoutType(String name) {
      try {
        layout =
            new AprilTagFieldLayout(
                Constants.disableHAL
                    ? Path.of("src", "main", "deploy", "apriltags", name + ".json")
                    : Path.of(
                        Filesystem.getDeployDirectory().getPath(), "apriltags", name + ".json"));
      } catch (IOException e) {
        throw new RuntimeException(e);
      }

      try {
        layoutString = new ObjectMapper().writeValueAsString(layout);
      } catch (JsonProcessingException e) {
        throw new RuntimeException(
            "Failed to serialize AprilTag layout JSON " + toString() + "for PhotonVision");
      }
    }

    private final AprilTagFieldLayout layout;
    private final String layoutString;
  }
}
