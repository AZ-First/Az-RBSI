// Copyright (c) 2024-2027 Az-FIRST
// http://github.com/AZ-First
// Copyright 2024 SleipnirGroup
// https://choreo.autos/
//
// Use of this source code is governed by a BSD
// license that can be found in the AdvantageKit-License.md file
// at the root directory of this project.

package frc.robot.commands;

import choreo.trajectory.SwerveSample;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Consumer;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.kinematics.ChassisVelocities;

/** Choreo swerve trajectory controller. */
public final class ChoreoAutoController implements Consumer<SwerveSample> {
  private final Drive drive;
  private final PIDController xController =
      new PIDController(
          AutoConstants.kChoreoDrivePID.kP,
          AutoConstants.kChoreoDrivePID.kI,
          AutoConstants.kChoreoDrivePID.kD);
  private final PIDController yController =
      new PIDController(
          AutoConstants.kChoreoDrivePID.kP,
          AutoConstants.kChoreoDrivePID.kI,
          AutoConstants.kChoreoDrivePID.kD);
  private final PIDController headingController =
      new PIDController(
          AutoConstants.kChoreoSteerPID.kP,
          AutoConstants.kChoreoSteerPID.kI,
          AutoConstants.kChoreoSteerPID.kD);

  public ChoreoAutoController(Drive drive) {
    this.drive = drive;
    headingController.enableContinuousInput(-Math.PI, Math.PI);
  }

  public void reset() {
    Pose2d pose = drive.getPose();
    xController.reset();
    yController.reset();
    headingController.reset();
    headingController.setSetpoint(pose.getRotation().getRadians());
  }

  @Override
  public void accept(SwerveSample referenceState) {
    Pose2d pose = drive.getPose();
    double xFeedback = xController.calculate(pose.getX(), referenceState.x);
    double yFeedback = yController.calculate(pose.getY(), referenceState.y);
    double rotationFeedback =
        headingController.calculate(pose.getRotation().getRadians(), referenceState.heading);

    ChassisVelocities output =
        new ChassisVelocities(
            referenceState.vx + xFeedback,
            referenceState.vy + yFeedback,
            referenceState.omega + rotationFeedback);
    drive.runVelocity(output.toRobotRelative(pose.getRotation()));
  }
}
