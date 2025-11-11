// Copyright (c) 2025 FRC 2486
// http://github.com/Coconuts2486-FRC
// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.DriveToPositionConstatnts.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.GeomUtil;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

/**
 * Drive to Pose Command
 *
 * <p>This command is used in both teleop and auto periods for autonomously driving the robot to a
 * desired pose.
 */
public class DriveToPose extends Command {

  private final Drive drive;
  private final Supplier<Pose2d> target;

  // Profiled PID controllers to run trapezoidal motion from current to desired pose
  private final ProfiledPIDController driveController =
      new ProfiledPIDController(
          0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0), Constants.loopPeriodSecs);
  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(
          0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0), Constants.loopPeriodSecs);

  // Various other values
  private Translation2d lastSetpointTranslation = Translation2d.kZero;
  private Rotation2d lastSetpointRotation = Rotation2d.kZero;
  private double lastTime = 0.0;
  private double driveErrorAbs = 0.0;
  private double thetaErrorAbs = 0.0;
  @Getter private boolean running = false;

  private Supplier<Translation2d> linearFF = () -> Translation2d.kZero;
  private DoubleSupplier omegaFF = () -> 0.0;

  /** Constructor */
  public DriveToPose(Drive drive, Supplier<Pose2d> target) {
    this.drive = drive;
    this.target = target;

    // Enable continuous input for theta controller
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // Lock-out Drive from other commands
    addRequirements(drive);
  }

  /** Initialize the Command */
  @Override
  public void initialize() {
    Pose2d currentPose = drive.getPose();
    ChassisSpeeds fieldVelocity =
        ChassisSpeeds.fromRobotRelativeSpeeds(
            drive.getChassisSpeeds(), drive.getPose().getRotation());
    Translation2d linearFieldVelocity =
        new Translation2d(fieldVelocity.vxMetersPerSecond, fieldVelocity.vyMetersPerSecond);
    driveController.reset(
        currentPose.getTranslation().getDistance(target.get().getTranslation()),
        Math.min(
            0.0,
            -linearFieldVelocity
                .rotateBy(
                    target
                        .get()
                        .getTranslation()
                        .minus(currentPose.getTranslation())
                        .getAngle()
                        .unaryMinus())
                .getX()));
    thetaController.reset(
        currentPose.getRotation().getRadians(), fieldVelocity.omegaRadiansPerSecond);
    lastSetpointTranslation = currentPose.getTranslation();
    lastSetpointRotation = target.get().getRotation();
    lastTime = Timer.getTimestamp();
  }

  /** Execute the command */
  @Override
  public void execute() {
    running = true;

    // Get current pose and target pose
    Pose2d currentPose = drive.getPose();
    Pose2d targetPose = target.get();

    // Calculate drive speed
    double currentDistance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
    double ffScaler =
        MathUtil.clamp((currentDistance - ffMinRadius) / (ffMaxRadius - ffMinRadius), 0.0, 1.0);
    driveErrorAbs = currentDistance;
    driveController.reset(
        lastSetpointTranslation.getDistance(targetPose.getTranslation()),
        driveController.getSetpoint().velocity);
    double driveVelocityScalar =
        driveController.calculate(driveErrorAbs, 0.0)
            + driveController.getSetpoint().velocity * ffScaler;
    if (currentDistance < driveController.getPositionTolerance()) driveVelocityScalar = 0.0;
    lastSetpointTranslation =
        new Pose2d(
                targetPose.getTranslation(),
                new Rotation2d(
                    Math.atan2(
                        currentPose.getTranslation().getY() - targetPose.getTranslation().getY(),
                        currentPose.getTranslation().getX() - targetPose.getTranslation().getX())))
            .transformBy(GeomUtil.toTransform2d(driveController.getSetpoint().position, 0.0))
            .getTranslation();

    // Calculate theta speed
    double thetaVelocity =
        thetaController.calculate(
                currentPose.getRotation().getRadians(),
                new TrapezoidProfile.State(
                    targetPose.getRotation().getRadians(),
                    (targetPose.getRotation().minus(lastSetpointRotation)).getRadians()
                        / (Timer.getTimestamp() - lastTime)))
            + thetaController.getSetpoint().velocity * ffScaler;
    thetaErrorAbs =
        Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getRadians());
    if (thetaErrorAbs < thetaController.getPositionTolerance()) thetaVelocity = 0.0;
    lastSetpointRotation = targetPose.getRotation();
    Translation2d driveVelocity =
        new Pose2d(
                Translation2d.kZero,
                new Rotation2d(
                    Math.atan2(
                        currentPose.getTranslation().getY() - targetPose.getTranslation().getY(),
                        currentPose.getTranslation().getX() - targetPose.getTranslation().getX())))
            .transformBy(GeomUtil.toTransform2d(driveVelocityScalar, 0.0))
            .getTranslation();
    lastTime = Timer.getTimestamp();

    // Scale feedback velocities by input ff
    final double linearS = linearFF.get().getNorm() * 3.0;
    final double thetaS = Math.abs(omegaFF.getAsDouble()) * 3.0;
    driveVelocity =
        driveVelocity.interpolate(
            linearFF.get().times(DrivebaseConstants.kMaxLinearSpeed), linearS);
    thetaVelocity =
        MathUtil.interpolate(
            thetaVelocity, omegaFF.getAsDouble() * DrivebaseConstants.kMaxAngularSpeed, thetaS);

    // Command speeds
    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            driveVelocity.getX(), driveVelocity.getY(), thetaVelocity, currentPose.getRotation()));

    // Log data
    Logger.recordOutput("DriveToPose/DistanceMeasured", currentDistance);
    Logger.recordOutput("DriveToPose/DistanceSetpoint", driveController.getSetpoint().position);
    Logger.recordOutput("DriveToPose/ThetaMeasured", currentPose.getRotation().getRadians());
    Logger.recordOutput("DriveToPose/ThetaSetpoint", thetaController.getSetpoint().position);
    Logger.recordOutput(
        "DriveToPose/Setpoint",
        new Pose2d[] {
          new Pose2d(
              lastSetpointTranslation,
              Rotation2d.fromRadians(thetaController.getSetpoint().position))
        });
    Logger.recordOutput("DriveToPose/Goal", new Pose2d[] {targetPose});
  }

  /** End the command */
  @Override
  public void end(boolean interrupted) {
    drive.stop();
    running = false;
    // Clear logs
    Logger.recordOutput("DriveToPose/Setpoint", new Pose2d[] {});
    Logger.recordOutput("DriveToPose/Goal", new Pose2d[] {});
  }

  /** Checks if the robot is stopped at the final pose. */
  public boolean atGoal() {
    return running && driveController.atGoal(); // && thetaController.atGoal();
  }

  /** Checks if the robot pose is within the allowed drive and theta tolerances. */
  public boolean withinTolerance(double driveTolerance, Rotation2d thetaTolerance) {
    return running
        && Math.abs(driveErrorAbs) < driveTolerance
        && Math.abs(thetaErrorAbs) < thetaTolerance.getRadians();
  }
}
