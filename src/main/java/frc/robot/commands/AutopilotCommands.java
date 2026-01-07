// Copyright (c) 2024-2026 Az-FIRST
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

package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import com.therekrab.autopilot.APTarget;
import com.therekrab.autopilot.Autopilot.APResult;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

public class AutopilotCommands {

  /**
   * AutoPilot Command Factory -- just reference pose
   *
   * @param drive The drivebase subsystem
   * @param reference The reference Pose2d at which the robot is to end
   * @return Command to run
   */
  public static Command runAutopilot(Drive drive, Pose2d reference) {

    APTarget target = new APTarget(reference);

    return autopilotToTarget(drive, target);
  }

  /**
   * AutoPilot Command Factory -- reference pose & entry angle
   *
   * @param drive The drivebase subsystem
   * @param reference The reference Pose2d at which the robot is to end
   * @param entryAngle The desired entry angle for arrival
   * @return Command to run
   */
  public static Command runAutopilot(Drive drive, Pose2d reference, Rotation2d entryAngle) {

    APTarget target = new APTarget(reference).withEntryAngle(entryAngle);

    return autopilotToTarget(drive, target);
  }

  /**
   * AutoPilot Command Factory -- reference pose & non-zero final velocity
   *
   * @param drive The drivebase subsystem
   * @param reference The reference Pose2d at which the robot is to end
   * @param velocity The desired end velocity when the robot approaches the target
   * @return Command to run
   */
  public static Command runAutopilot(Drive drive, Pose2d reference, double velocity) {

    APTarget target = new APTarget(reference).withVelocity(velocity);

    return autopilotToTarget(drive, target);
  }

  /**
   * AutoPilot Command Factory -- reference pose & entry angle & non-zero final velocity
   *
   * @param drive The drivebase subsystem
   * @param reference The reference Pose2d at which the robot is to end
   * @param entryAngle The desired entry angle for arrival
   * @param velocity The desired end velocity when the robot approaches the target
   * @return Command to run
   */
  public static Command runAutopilot(
      Drive drive, Pose2d reference, Rotation2d entryAngle, double velocity) {

    APTarget target = new APTarget(reference).withEntryAngle(entryAngle).withVelocity(velocity);

    return autopilotToTarget(drive, target);
  }

  /**
   * AutoPilot Command Factory -- reference pose & rotation radius
   *
   * @param drive The drivebase subsystem
   * @param reference The reference Pose2d at which the robot is to end
   * @param radius The distance from the target pose that rotation goals are respected
   * @return Command to run
   */
  public static Command runAutopilot(Drive drive, Pose2d reference, Distance radius) {

    APTarget target = new APTarget(reference).withRotationRadius(radius);

    return autopilotToTarget(drive, target);
  }

  /**
   * AutoPilot Command Factory -- reference pose & entry angle & rotation radius
   *
   * @param drive The drivebase subsystem
   * @param reference The reference Pose2d at which the robot is to end
   * @param entryAngle The desired entry angle for arrival
   * @param radius The distance from the target pose that rotation goals are respected
   * @return Command to run
   */
  public static Command runAutopilot(
      Drive drive, Pose2d reference, Rotation2d entryAngle, Distance radius) {

    APTarget target = new APTarget(reference).withEntryAngle(entryAngle).withRotationRadius(radius);

    return autopilotToTarget(drive, target);
  }

  /**
   * AutoPilot Command Factory -- reference pose & non-zero final velocity & rotation radius
   *
   * @param drive The drivebase subsystem
   * @param reference The reference Pose2d at which the robot is to end
   * @param velocity The desired end velocity when the robot approaches the target
   * @param radius The distance from the target pose that rotation goals are respected
   * @return Command to run
   */
  public static Command runAutopilot(
      Drive drive, Pose2d reference, double velocity, Distance radius) {

    APTarget target = new APTarget(reference).withVelocity(velocity).withRotationRadius(radius);

    return autopilotToTarget(drive, target);
  }

  /**
   * AutoPilot Command Factory -- reference pose & entry angle & non-zero final velocity & rotation
   * radius
   *
   * @param drive The drivebase subsystem
   * @param reference The reference Pose2d at which the robot is to end
   * @param entryAngle The desired entry angle for arrival
   * @param velocity The desired end velocity when the robot approaches the target
   * @param radius The distance from the target pose that rotation goals are respected
   * @return Command to run
   */
  public static Command runAutopilot(
      Drive drive, Pose2d reference, Rotation2d entryAngle, double velocity, Distance radius) {

    APTarget target =
        new APTarget(reference)
            .withEntryAngle(entryAngle)
            .withVelocity(velocity)
            .withRotationRadius(radius);

    return autopilotToTarget(drive, target);
  }

  /**
   * Underlying private command that takes a APTarget formulated by the public commands
   *
   * @param drive The drivebase subsystem
   * @param target The formulated Autopilot Target
   * @return Command to run
   */
  private static Command autopilotToTarget(Drive drive, APTarget target) {

    // Create PID controller
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            AutoConstants.kPPsteerPID.kP,
            AutoConstants.kPPsteerPID.kI,
            AutoConstants.kPPsteerPID.kD,
            new TrapezoidProfile.Constraints(
                DrivebaseConstants.kMaxAngularSpeed, DrivebaseConstants.kMaxAngularAccel));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    return Commands.run(
            () -> {
              ChassisSpeeds robotRelativeSpeeds = drive.getChassisSpeeds();
              Pose2d pose = drive.getPose();

              Logger.recordOutput("Autopilot/CurrentPose", pose);
              Logger.recordOutput("Autopilot/FinalPose", target.getReference());
              Logger.recordOutput("Autopilot/RobotSpeeds", robotRelativeSpeeds);

              // Compute the needed output control transform to move the robot to the desired
              // position
              APResult output =
                  AutoConstants.kAutopilot.calculate(pose, robotRelativeSpeeds, target);

              Logger.recordOutput("Autopilot/outputVx", output.vx());
              Logger.recordOutput("Autopilot/outputVy", output.vy());
              Logger.recordOutput("Autopilot/targetAngle", output.targetAngle());
              Logger.recordOutput(
                  "Autopilot/atTarget", AutoConstants.kAutopilot.atTarget(drive.getPose(), target));

              // Output is field relative
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      output.vx(),
                      output.vy(),
                      RadiansPerSecond.of(
                          angleController.calculate(
                              drive.getHeading().getRadians(), output.targetAngle().getRadians())));

              drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, drive.getHeading()));
            },
            drive)

        // Reset PID controller when command starts
        .beforeStarting(() -> angleController.reset(drive.getHeading().getRadians()))
        .until(() -> AutoConstants.kAutopilot.atTarget(drive.getPose(), target));
  }
}
