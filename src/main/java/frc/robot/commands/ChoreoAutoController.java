// Copyright (c) 2024-2026 Az-FIRST
// http://github.com/AZ-First
// Copyright 2024 SleipnirGroup
// https://choreo.autos/
//
// Use of this source code is governed by a BSD
// license that can be found in the AdvantageKit-License.md file
// at the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.AutoConstants.*;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Consumer;

public class ChoreoAutoController implements Consumer<SwerveSample> {
  private final Drive drive; // drive subsystem
  private final PIDController xController =
      new PIDController(kChoreoDrivePID.kP, kChoreoDrivePID.kI, kChoreoDrivePID.kD);
  private final PIDController yController =
      new PIDController(kChoreoDrivePID.kP, kChoreoDrivePID.kI, kChoreoDrivePID.kD);
  private final PIDController headingController =
      new PIDController(kChoreoSteerPID.kP, kChoreoSteerPID.kI, kChoreoSteerPID.kD);

  public ChoreoAutoController(Drive drive) {
    this.drive = drive;
    headingController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void accept(SwerveSample referenceState) {
    Pose2d pose = drive.getPose();
    double xFF = referenceState.vx;
    double yFF = referenceState.vy;
    double rotationFF = referenceState.omega;

    double xFeedback = xController.calculate(pose.getX(), referenceState.x);
    double yFeedback = yController.calculate(pose.getY(), referenceState.y);
    double rotationFeedback =
        headingController.calculate(pose.getRotation().getRadians(), referenceState.heading);

    // Convert to field relative speeds & send command
    ChassisSpeeds out =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            xFF + xFeedback, yFF + yFeedback, rotationFF + rotationFeedback, pose.getRotation());
    drive.runVelocity(out);
  }
}
