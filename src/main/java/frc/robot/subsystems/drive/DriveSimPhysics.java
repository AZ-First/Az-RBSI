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

package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * Authoritative drivetrain physics model for SIM.
 *
 * <p>Responsibilities: - Integrates yaw with inertia + damping - Integrates translation in the
 * correct frame - Produces the ONE true robot pose for SIM
 */
public class DriveSimPhysics {

  /* ---------------- Physical state ---------------- */
  private Pose2d pose = Pose2d.kZero;
  private double omegaRadPerSec = 0.0;
  private Translation2d prevVelocity = new Translation2d(0, 0);
  private Translation2d linearAccel = new Translation2d(0, 0);

  /* ---------------- Robot constants ---------------- */
  private final double moiKgMetersSq;
  private final double maxTorqueNm;
  private static final double ANGULAR_DAMPING = 4.0; // Nm per rad/sec

  private final SwerveDriveKinematics kinematics;

  public DriveSimPhysics(
      SwerveDriveKinematics kinematics, double moiKgMetersSq, double maxTorqueNm) {
    this.kinematics = kinematics;
    this.moiKgMetersSq = moiKgMetersSq;
    this.maxTorqueNm = maxTorqueNm;
  }

  /** Integrate physics from module states (SIM-only) */
  public void update(SwerveModuleState[] moduleStates, double dtSeconds) {

    // ------------------ LINEAR ------------------
    ChassisSpeeds chassis = kinematics.toChassisSpeeds(moduleStates);
    Translation2d fieldVelocity =
        new Translation2d(chassis.vxMetersPerSecond, chassis.vyMetersPerSecond);

    // Finite-difference linear acceleration for IMU simulation
    linearAccel = fieldVelocity.minus(prevVelocity).div(dtSeconds);
    prevVelocity = fieldVelocity;

    Translation2d newTranslation = pose.getTranslation().plus(fieldVelocity.times(dtSeconds));

    // ------------------ ANGULAR ------------------
    double commandedOmega = chassis.omegaRadiansPerSecond;

    // Convert commanded angular velocity to torque (simple capped model)
    double torque =
        MathUtil.clamp(commandedOmega * moiKgMetersSq / dtSeconds, -maxTorqueNm, maxTorqueNm);

    // Integrate angular velocity with damping
    omegaRadPerSec += (torque / moiKgMetersSq - ANGULAR_DAMPING * omegaRadPerSec) * dtSeconds;

    // Integrate yaw
    Rotation2d newYaw = pose.getRotation().plus(new Rotation2d(omegaRadPerSec * dtSeconds));

    // ------------------ FINAL POSE ------------------
    pose = new Pose2d(newTranslation, newYaw);
  }

  /* ================== Getters ================== */
  public Pose2d getPose() {
    return pose;
  }

  public Rotation2d getYaw() {
    return pose.getRotation();
  }

  public double getOmegaRadPerSec() {
    return omegaRadPerSec;
  }

  public Translation2d getLinearAccel() {
    return linearAccel;
  }

  /* ================== Reset ================== */
  public void reset(Pose2d pose) {
    this.pose = pose;
    this.omegaRadPerSec = 0.0;
    this.prevVelocity = new Translation2d();
    this.linearAccel = new Translation2d();
  }
}
