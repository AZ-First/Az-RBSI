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

import static org.wpilib.units.Units.*;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import lombok.Getter;
import org.wpilib.system.Filesystem;
import org.wpilib.vision.apriltag.AprilTagFieldLayout;

/**
 * Contains various field dimensions and useful reference points. All units are in meters and poses
 * have a blue alliance origin.
 */
public class FieldConstants {

  /** Specify which type of field your robot is using */
  private static final AprilTagLayoutType fieldType = AprilTagLayoutType.REBUILT_WELDED;

  /** AprilTag Field Layout ************************************************ */
  public static final double aprilTagWidth = Inches.of(6.50).in(Meters);

  public static final String aprilTagFamily = "36h11";

  public static final double fieldLength = fieldType.getLayout().getFieldLength();
  public static final double fieldWidth = fieldType.getLayout().getFieldWidth();

  public static final int aprilTagCount = fieldType.getLayout().getTags().size();
  public static final AprilTagLayoutType defaultAprilTagType = fieldType;

  public static final AprilTagFieldLayout aprilTagLayout = fieldType.getLayout();

  @Getter
  public enum AprilTagLayoutType {
    REBUILT_WELDED("2026-rebuilt-welded"),
    REBUILT_ANDYMARK("2026-rebuilt-andymark"),
    REEFSCAPE_WELDED("2025-reefscape-welded"),
    REEFSCAPE_ANDYMARK("2025-reefscape-andymark"),
    CRESCENDO("2024-crescendo"),
    NONE_WELDED("none-welded"),
    NONE_ANDYMARK("none-andymark");

    AprilTagLayoutType(String name) {
      Path layoutPath =
          Constants.disableHAL
              ? Path.of("src", "main", "deploy", "apriltags", name + ".json")
              : Path.of(Filesystem.getDeployDirectory().getPath(), "apriltags", name + ".json");

      try {
        layout = new AprilTagFieldLayout(layoutPath);
      } catch (IOException e) {
        throw new RuntimeException(e);
      }

      try {
        layoutString = Files.readString(layoutPath);
      } catch (IOException e) {
        throw new RuntimeException(
            "Failed to read AprilTag layout JSON " + toString() + " for PhotonVision", e);
      }
    }

    private final AprilTagFieldLayout layout;
    private final String layoutString;
  }
}
