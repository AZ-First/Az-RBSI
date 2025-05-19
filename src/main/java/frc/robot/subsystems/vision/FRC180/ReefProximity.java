// Copyright (c) 2025 FRC 180
// https://github.com/frc180
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

package frc.robot.subsystems.vision.FRC180;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map.Entry;

/**
 * This class is used to determine the closest reef scoring position to the robot's current
 * position.
 */
public class ReefProximity {

  final List<Entry<Integer, Pose2d>> reefList;

  public ReefProximity(
      HashMap<Integer, Pose2d> leftReefHashMap, HashMap<Integer, Pose2d> rightReefHashMap) {
    reefList = new ArrayList<Entry<Integer, Pose2d>>();
    leftReefHashMap.entrySet().forEach(reefList::add);
    rightReefHashMap.entrySet().forEach(reefList::add);
  }

  public Entry<Integer, Pose2d> closestReefPose(Pose2d position, boolean blueAlliance) {
    return closestReefPose(
        position, blueAlliance ? VisionSubsystem.blueReefTags : VisionSubsystem.redReefTags);
  }

  public Entry<Integer, Pose2d> closestReefPose(Pose2d position, List<Integer> tagOptions) {
    double currentDistance = Double.MAX_VALUE;
    Entry<Integer, Pose2d> closest = null;

    for (var entry : reefList) {
      // Ensure the reef tag is in the list of tags we are looking for
      if (!tagOptions.contains(entry.getKey())) {
        continue;
      }

      double distance = position.getTranslation().getDistance(entry.getValue().getTranslation());
      if (distance < currentDistance) {
        currentDistance = distance;
        closest = entry;
      }
    }

    return closest;
  }
}
