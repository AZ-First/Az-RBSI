// Copyright (c) 2024 Az-FIRST
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
//
// NOTE: This module based on the YAGSL Example Project

package frc.robot.subsystems.swervedrive.underlying;

import static frc.robot.Constants.DrivebaseConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.Constants.AutonConstants;
import frc.robot.subsystems.vision.Vision;
import java.io.File;
import java.util.Arrays;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

/** YAGSLSwerve module */
public class YAGSLSwerve extends SubsystemBase {

  /** Swerve drive object */
  private final SwerveDrive swerveDrive;

  /** PhotonVision class to keep an accurate odometry */
  private Vision vision;

  /** Enable vision odometry updates while driving */
  private final boolean visionDriveTest = false;

  /**
   * Initialize {@link SwerveDrive} with the directory provided.
   *
   * @param directory Directory of swerve drive config files.
   */
  public YAGSLSwerve(File directory) {

    // Angle conversion factor is 360 / (GEAR RATIO * ENCODER RESOLUTION)
    double angleConversionFactor = SwerveMath.calculateDegreesPerSteeringRotation(TURN_GEAR_RATIO);

    // Motor conversion factor is (PI * WHEEL DIAMETER IN METERS) / (GEAR RATIO * ENCODER
    // RESOLUTION)
    double driveConversionFactor =
        SwerveMath.calculateMetersPerRotation(WHEEL_RADIUS * 2.0, DRIVE_GEAR_RATIO);

    // Output to console and to AdvantageKit
    System.out.println("\"conversionFactors\": {");
    System.out.println("\t\"angle\": {\"factor\": " + angleConversionFactor + " },");
    System.out.println("\t\"drive\": {\"factor\": " + driveConversionFactor + " }");
    System.out.println("}");
    Logger.recordOutput("SwerveDive/ConversionFactors/Angle", angleConversionFactor);
    Logger.recordOutput("SwerveDive/ConversionFactors/Drive", driveConversionFactor);

    // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary objects being
    // created.
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    try {
      swerveDrive = new SwerveParser(directory).createSwerveDrive(MAX_LINEAR_SPEED);
      // Alternative method if you don't want to supply the conversion factor via JSON files.
      // swerveDrive = new SwerveParser_RBSI(directory).createSwerveDrive(maximumSpeed,
      // angleConversionFactor, driveConversionFactor);
    } catch (Exception e) {
      throw new RuntimeException(e);
    }
    // Heading correction should only be used while controlling the robot via angle.
    swerveDrive.setHeadingCorrection(false);
    // Disables cosine compensation for simulations since it causes discrepancies not seen in real
    // life.
    if (SwerveDriveTelemetry.isSimulation) {
      swerveDrive.setCosineCompensator(false);
    } else {
      swerveDrive.setCosineCompensator(true);
    }

    // Correct for skew that gets worse as angular velocity increases. Start with a coefficient of
    // 0.1
    swerveDrive.setAngularVelocityCompensation(true, true, 0.1);
    // Enable if you want to resynchronize your absolute encoders and motor encoders periodically
    // when they are not moving.
    swerveDrive.setModuleEncoderAutoSynchronize(false, 1);
    // Set the absolute encoder to be used over the internal encoder and push the offsets onto it.
    // Throws warning if not possible
    swerveDrive.pushOffsetsToEncoders();

    if (visionDriveTest) {
      setupPhotonVision();
      // Stop the odometry thread if we are using vision that way we can synchronize updates better.
      swerveDrive.stopOdometryThread();
    }
    setupPathPlanner();
  }

  /**
   * Construct the swerve drive.
   *
   * @param driveCfg SwerveDriveConfiguration for the swerve.
   * @param controllerCfg Swerve Controller.
   */
  public YAGSLSwerve(
      SwerveDriveConfiguration driveCfg, SwerveControllerConfiguration controllerCfg) {
    swerveDrive = new SwerveDrive(driveCfg, controllerCfg, MAX_LINEAR_SPEED);
  }

  /** Setup the photon vision class. */
  public void setupPhotonVision() {
    vision = new Vision(swerveDrive::getPose, swerveDrive.field);
  }

  /** Setup AutoBuilder for PathPlanner. */
  public void setupPathPlanner() {
    AutoBuilder.configureHolonomic(
        this::getPose, // Robot pose supplier
        this::resetOdometry, // Method to reset odometry (called if your auto has a starting pose)
        this::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::setChassisSpeeds, // Driver method given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig(
            AutonConstants.TRANSLATION_PID,
            AutonConstants.ANGLE_PID,
            MAX_LINEAR_SPEED,
            DRIVE_BASE_RADIUS,
            new ReplanningConfig()),
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
          var alliance = DriverStation.getAlliance();
          return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
        },
        this // Reference to this subsystem to set requirements
        );
  }

  /** Periodic function -- update odometry and log everything */
  @Override
  public void periodic() {
    // When vision is enabled we must manually update odometry in SwerveDrive
    if (visionDriveTest) {
      swerveDrive.updateOdometry();
      vision.updatePoseEstimation(swerveDrive);
    }

    /** Log Telemetry Data to AdvantageKit */
    Logger.recordOutput("SwerveDive/Telemetry/moduleCount", SwerveDriveTelemetry.moduleCount);
    Logger.recordOutput("SwerveDive/Telemetry/wheelLocations", SwerveDriveTelemetry.wheelLocations);
    Logger.recordOutput("SwerveDive/Telemetry/measuredStates", SwerveDriveTelemetry.measuredStates);
    Logger.recordOutput("SwerveDive/Telemetry/desiredStates", SwerveDriveTelemetry.desiredStates);
    Logger.recordOutput("SwerveDive/Telemetry/robotRotation", SwerveDriveTelemetry.robotRotation);
    Logger.recordOutput("SwerveDive/Telemetry/maxSpeed", SwerveDriveTelemetry.maxSpeed);
    Logger.recordOutput("SwerveDive/Telemetry/rotationUnit", SwerveDriveTelemetry.rotationUnit);
    Logger.recordOutput("SwerveDive/Telemetry/sizeLeftRight", SwerveDriveTelemetry.sizeLeftRight);
    Logger.recordOutput("SwerveDive/Telemetry/sizeFrontBack", SwerveDriveTelemetry.sizeFrontBack);
    Logger.recordOutput(
        "SwerveDive/Telemetry/forwardDirection", SwerveDriveTelemetry.forwardDirection);
    Logger.recordOutput(
        "SwerveDive/Telemetry/maxAngularVelocity", SwerveDriveTelemetry.maxAngularVelocity);
    Logger.recordOutput(
        "SwerveDive/Telemetry/measuredChassisSpeeds", SwerveDriveTelemetry.measuredChassisSpeeds);
    Logger.recordOutput(
        "SwerveDive/Telemetry/desiredChassisSpeeds", SwerveDriveTelemetry.desiredChassisSpeeds);

    /** Log Swerve Drive States to AdvantageKit */
    getModuleStates();
    getDesiredStates();
    Logger.recordOutput(
        "SwerveDive/States/RobotRotation",
        SwerveDriveTelemetry.rotationUnit == "degrees"
            ? Rotation2d.fromDegrees(SwerveDriveTelemetry.robotRotation)
            : Rotation2d.fromRadians(SwerveDriveTelemetry.robotRotation));
  }

  /** Simulation periodic function */
  @Override
  public void simulationPeriodic() {}

  /**
   * The primary method for controlling the drivebase. Takes a {@link Translation2d} and a rotation
   * rate, and calculates and commands module states accordingly. Can use either open-loop or
   * closed-loop velocity control for the wheel velocities. Also has field- and robot-relative
   * modes, which affect how the translation vector is used.
   *
   * @param translation {@link Translation2d} that is the commanded linear velocity of the robot, in
   *     meters per second. In robot-relative mode, positive x is torwards the bow (front) and
   *     positive y is torwards port (left). In field-relative mode, positive x is away from the
   *     alliance wall (field North) and positive y is torwards the left wall when looking through
   *     the driver station glass (field West).
   * @param rotation Robot angular rate, in radians per second. CCW positive. Unaffected by
   *     field/robot relativity.
   * @param fieldRelative Drive mode. True for field-relative, false for robot-relative.
   */
  public void drive(Translation2d translation, double rotation, boolean fieldRelative) {

    // Open loop is disabled since it shouldn't be used most of the time.
    swerveDrive.drive(translation, rotation, fieldRelative, false);
  }

  /**
   * Drive the robot given a chassis field oriented velocity.
   *
   * @param velocity Velocity according to the field.
   */
  public void driveFieldOriented(ChassisSpeeds velocity) {
    swerveDrive.driveFieldOriented(velocity);
  }

  /**
   * Drive according to the chassis robot oriented velocity.
   *
   * @param velocity Robot oriented {@link ChassisSpeeds}
   */
  public void drive(ChassisSpeeds velocity) {
    swerveDrive.drive(velocity);
  }

  /************************************************************************* */
  /* COMMAND SECTION -- Swerve-only Commands */

  /**
   * Command to drive the robot using translative values and heading as a setpoint.
   *
   * @param translationX Translation in the X direction. Cubed for smoother controls.
   * @param translationY Translation in the Y direction. Cubed for smoother controls.
   * @param headingX Heading X to calculate angle of the joystick.
   * @param headingY Heading Y to calculate angle of the joystick.
   * @return Drive command.
   */
  public Command driveCommand(
      DoubleSupplier translationX,
      DoubleSupplier translationY,
      DoubleSupplier headingX,
      DoubleSupplier headingY) {
    // swerveDrive.setHeadingCorrection(true); // Normally you would want heading correction for
    // this kind of control.
    return run(
        () -> {
          Translation2d scaledInputs =
              SwerveMath.scaleTranslation(
                  new Translation2d(translationX.getAsDouble(), translationY.getAsDouble()), 0.8);

          // Make the robot move
          driveFieldOriented(
              swerveDrive.swerveController.getTargetSpeeds(
                  scaledInputs.getX(),
                  scaledInputs.getY(),
                  headingX.getAsDouble(),
                  headingY.getAsDouble(),
                  swerveDrive.getOdometryHeading().getRadians(),
                  swerveDrive.getMaximumVelocity()));
        });
  }

  /**
   * Command to drive the robot using translative values and heading as angular velocity. NOTE:
   * Alternate drive command constructor
   *
   * @param translationX Translation in the X direction. Cubed for smoother controls.
   * @param translationY Translation in the Y direction. Cubed for smoother controls.
   * @param angularRotationX Angular velocity of the robot to set. Cubed for smoother controls.
   * @return Drive command.
   */
  public Command driveCommand(
      DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX) {
    return run(
        () -> {
          // Make the robot move
          swerveDrive.drive(
              SwerveMath.scaleTranslation(
                  new Translation2d(
                      translationX.getAsDouble() * swerveDrive.getMaximumVelocity(),
                      translationY.getAsDouble() * swerveDrive.getMaximumVelocity()),
                  0.8),
              Math.pow(angularRotationX.getAsDouble(), 3) * swerveDrive.getMaximumAngularVelocity(),
              true,
              false);
        });
  }

  /**
   * Command to drive the robot using translative values and heading as a setpoint.
   *
   * @param translationX Translation in the X direction.
   * @param translationY Translation in the Y direction.
   * @param rotation Rotation as a value between [-1, 1] converted to radians.
   * @return Drive command.
   */
  public Command simDriveCommand(
      DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier rotation) {
    // swerveDrive.setHeadingCorrection(true); // Normally you would want heading correction for
    // this kind of control.
    return run(
        () -> {
          // Make the robot move
          driveFieldOriented(
              swerveDrive.swerveController.getTargetSpeeds(
                  translationX.getAsDouble(),
                  translationY.getAsDouble(),
                  rotation.getAsDouble() * Math.PI,
                  swerveDrive.getOdometryHeading().getRadians(),
                  swerveDrive.getMaximumVelocity()));
        });
  }

  /**
   * Get the path follower with events.
   *
   * @param pathName PathPlanner path name.
   * @return {@link AutoBuilder#followPath(PathPlannerPath)} path command.
   */
  public Command getAutonomousCommand(String pathName) {
    // Create a path following command using AutoBuilder. This will also trigger event markers.
    return new PathPlannerAuto(pathName);
  }

  /**
   * Use PathPlanner Path finding to go to a point on the field.
   *
   * @param pose Target {@link Pose2d} to go to.
   * @return PathFinding command
   */
  public Command driveToPose(Pose2d pose) {
    // Create the constraints to use while pathfinding
    PathConstraints constraints =
        new PathConstraints(
            MAX_LINEAR_SPEED, MAX_LINEAR_ACCEL, MAX_ANGULAR_SPEED, MAX_ANGULAR_ACCEL);

    // Since AutoBuilder is configured, we can use it to build pathfinding commands
    return AutoBuilder.pathfindToPose(
        pose,
        constraints,
        0.0, // Goal end velocity in meters/sec
        0.0 // Rotation delay distance in meters. This is how far the robot should travel before
        // attempting to rotate.
        );
  }

  /**
   * Command to characterize the robot drive motors using SysId
   *
   * @return SysId Drive Command
   */
  public Command sysIdDriveMotorCommand() {
    return SwerveDriveTest.generateSysIdCommand(
        SwerveDriveTest.setDriveSysIdRoutine(new Config(), this, swerveDrive, kMaxV),
        kDelay,
        kQuasiTimeout,
        kDynamicTimeout);
  }

  /**
   * Command to characterize the robot angle motors using SysId
   *
   * @return SysId Angle Command
   */
  public Command sysIdAngleMotorCommand() {
    return SwerveDriveTest.generateSysIdCommand(
        SwerveDriveTest.setAngleSysIdRoutine(new Config(), this, swerveDrive),
        kDelay,
        kQuasiTimeout,
        kDynamicTimeout);
  }

  /**
   * Returns a Command that centers the modules of the SwerveDrive subsystem.
   *
   * @return a Command that centers the modules of the SwerveDrive subsystem
   */
  public Command centerModulesCommand() {
    return run(() -> Arrays.asList(swerveDrive.getModules()).forEach(it -> it.setAngle(0.0)));
  }

  /**
   * Returns a Command that drives the swerve drive to a specific distance at a given speed.
   *
   * @param distanceInMeters the distance to drive in meters
   * @param speedInMetersPerSecond the speed at which to drive in meters per second
   * @return a Command that drives the swerve drive to a specific distance at a given speed
   */
  public Command driveToDistanceCommand(double distanceInMeters, double speedInMetersPerSecond) {
    return Commands.deferredProxy(
        () ->
            Commands.run(() -> drive(new ChassisSpeeds(speedInMetersPerSecond, 0, 0)), this)
                .until(
                    () ->
                        swerveDrive.getPose().getTranslation().getDistance(new Translation2d(0, 0))
                            > distanceInMeters));
  }

  /************************************************************************* */
  /* UTILITY SECTION -- Utility methods */

  /**
   * Get the swerve drive kinematics object.
   *
   * @return {@link SwerveDriveKinematics} of the swerve drive.
   */
  @AutoLogOutput(key = "Odometry/Kinematics")
  public SwerveDriveKinematics getKinematics() {
    return swerveDrive.kinematics;
  }

  /**
   * Gets the current pose (position and rotation) of the robot, as reported by odometry.
   *
   * @return The robot's pose
   */
  @AutoLogOutput(key = "Odometry/RobotPose")
  public Pose2d getPose() {
    return swerveDrive.getPose();
  }

  /**
   * Gets the current yaw angle of the robot, as reported by the swerve pose estimator in the
   * underlying drivebase. Note, this is not the raw gyro reading, this may be corrected from calls
   * to resetOdometry().
   *
   * @return The yaw angle
   */
  @AutoLogOutput(key = "Odometry/Heading")
  public Rotation2d getHeading() {
    return getPose().getRotation();
  }

  /**
   * Gets the current field-relative velocity (x, y and omega) of the robot
   *
   * @return A ChassisSpeeds object of the current field-relative velocity
   */
  @AutoLogOutput(key = "Odometry/FieldVelocity")
  public ChassisSpeeds getFieldVelocity() {
    return swerveDrive.getFieldVelocity();
  }

  /**
   * Gets the current velocity (x, y and omega) of the robot
   *
   * @return A {@link ChassisSpeeds} object of the current velocity
   */
  @AutoLogOutput(key = "Odometry/RobotVelocity")
  public ChassisSpeeds getRobotVelocity() {
    return swerveDrive.getRobotVelocity();
  }

  /**
   * Gets the current pitch angle of the robot, as reported by the imu.
   *
   * @return The heading as a {@link Rotation2d} angle
   */
  @AutoLogOutput(key = "Odometry/Pitch")
  public Rotation2d getPitch() {
    return swerveDrive.getPitch();
  }

  /**
   * Set chassis speeds with closed-loop velocity control.
   *
   * @param chassisSpeeds Chassis Speeds to set.
   */
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    swerveDrive.setChassisSpeeds(chassisSpeeds);
  }

  /**
   * Resets odometry to the given pose. Gyro angle and module positions do not need to be reset when
   * calling this method. However, if either gyro angle or module position is reset, this must be
   * called in order for odometry to keep working.
   *
   * @param initialHolonomicPose The pose to set the odometry to
   */
  public void resetOdometry(Pose2d initialHolonomicPose) {
    swerveDrive.resetOdometry(initialHolonomicPose);
  }

  /**
   * Post the trajectory to the field.
   *
   * @param trajectory The trajectory to post.
   */
  public void postTrajectory(Trajectory trajectory) {
    swerveDrive.postTrajectory(trajectory);
  }

  /**
   * Resets the gyro angle to zero and resets odometry to the same position, but facing toward 0.
   */
  public void zeroGyro() {
    swerveDrive.zeroGyro();
  }

  /**
   * Checks if the alliance is red, defaults to false if alliance isn't available.
   *
   * @return true if the red alliance, false if blue. Defaults to false if none is available.
   */
  private boolean isRedAlliance() {
    var alliance = DriverStation.getAlliance();
    return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
  }

  /**
   * This will zero (calibrate) the robot to assume the current position is facing forward
   *
   * <p>If red alliance rotate the robot 180 after the drviebase zero command
   */
  public void zeroGyroWithAlliance() {
    if (isRedAlliance()) {
      zeroGyro();
      // Set the pose 180 degrees
      resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
    } else {
      zeroGyro();
    }
  }

  /**
   * Sets the drive motors to brake/coast mode.
   *
   * @param brake True to set motors to brake mode, false for coast.
   */
  public void setMotorBrake(boolean brake) {
    swerveDrive.setMotorIdleMode(brake);
  }

  /**
   * Get the chassis speeds based on controller input of 2 joysticks. One for speeds in which
   * direction. The other for the angle of the robot.
   *
   * @param xInput X joystick input for the robot to move in the X direction.
   * @param yInput Y joystick input for the robot to move in the Y direction.
   * @param headingX X joystick which controls the angle of the robot.
   * @param headingY Y joystick which controls the angle of the robot.
   * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(
      double xInput, double yInput, double headingX, double headingY) {
    Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));
    return swerveDrive.swerveController.getTargetSpeeds(
        scaledInputs.getX(),
        scaledInputs.getY(),
        headingX,
        headingY,
        getHeading().getRadians(),
        swerveDrive.getMaximumVelocity());
  }

  /**
   * Get the chassis speeds based on controller input of 1 joystick and one angle. Control the robot
   * at an offset of 90deg.
   *
   * @param xInput X joystick input for the robot to move in the X direction.
   * @param yInput Y joystick input for the robot to move in the Y direction.
   * @param angle The angle in as a {@link Rotation2d}.
   * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle) {
    Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));

    return swerveDrive.swerveController.getTargetSpeeds(
        scaledInputs.getX(),
        scaledInputs.getY(),
        angle.getRadians(),
        getHeading().getRadians(),
        swerveDrive.getMaximumVelocity());
  }

  /**
   * Get the {@link SwerveController} in the swerve drive.
   *
   * @return {@link SwerveController} from the {@link SwerveDrive}.
   */
  public SwerveController getSwerveController() {
    return swerveDrive.swerveController;
  }

  /**
   * Get the {@link SwerveDriveConfiguration} object.
   *
   * @return The {@link SwerveDriveConfiguration} for the current drive.
   */
  @AutoLogOutput(key = "SwerveDrive/Config")
  public SwerveDriveConfiguration getSwerveDriveConfiguration() {
    return swerveDrive.swerveDriveConfiguration;
  }

  /** Lock the swerve drive to prevent it from moving. */
  public void lock() {
    swerveDrive.lockPose();
  }

  /** Add a fake vision reading for testing purposes. */
  public void addFakeVisionReading() {
    swerveDrive.addVisionMeasurement(
        new Pose2d(3, 3, Rotation2d.fromDegrees(65)), Timer.getFPGATimestamp());
  }

  /**
   * Sets the maximum speed of the swerve drive.
   *
   * @param maximumSpeedInMetersPerSecond the maximum speed to set for the swerve drive in meters
   *     per second
   */
  public void setMaximumSpeed(double maximumSpeedInMetersPerSecond) {
    swerveDrive.setMaximumSpeed(
        maximumSpeedInMetersPerSecond,
        false,
        swerveDrive.swerveDriveConfiguration.physicalCharacteristics.optimalVoltage);
  }

  /**
   * Replaces the swerve module feedforward with a new SimpleMotorFeedforward object.
   *
   * @param kS the static gain of the feedforward
   * @param kV the velocity gain of the feedforward
   * @param kA the acceleration gain of the feedforward
   */
  public void replaceSwerveModuleFeedforward(double kS, double kV, double kA) {
    swerveDrive.replaceSwerveModuleFeedforward(new SimpleMotorFeedforward(kS, kV, kA));
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveDive/States/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] =
          new SwerveModuleState(
              SwerveDriveTelemetry.measuredStates[(i * 2) + 1],
              Rotation2d.fromDegrees(SwerveDriveTelemetry.measuredStates[i * 2]));
    }
    return states;
  }

  /** Returns the desired states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveDive/States/Desired")
  private SwerveModuleState[] getDesiredStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] =
          new SwerveModuleState(
              SwerveDriveTelemetry.desiredStates[(i * 2) + 1],
              Rotation2d.fromDegrees(SwerveDriveTelemetry.desiredStates[i * 2]));
    }
    return states;
  }

  /************************************************************************* */
  /** 2024 SEASON-SPECIFIC FUNCTIONS, INCLUDED AS EXAMPLES */

  /**
   * Aim the robot at the speaker.
   *
   * @param tolerance Tolerance in degrees.
   * @return Command to turn the robot to the speaker.
   */
  public Command aimAtSpeaker(double tolerance) {
    SwerveController controller = swerveDrive.getSwerveController();
    return run(() -> {
          drive(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  0,
                  0,
                  controller.headingCalculate(
                      getHeading().getRadians(),
                      vision.getSpeakerYaw(swerveDrive.getOdometryHeading()).getRadians()),
                  getHeading()));
        })
        .until(
            () ->
                Math.abs(
                        vision
                            .getSpeakerYaw(swerveDrive.getOdometryHeading())
                            .minus(getHeading())
                            .getDegrees())
                    < tolerance);
  }

  /**
   * Aim the robot at the target returned by PhotonVision.
   *
   * @param camera {@link PhotonCamera} to communicate with.
   * @return A {@link Command} which will run the alignment.
   */
  public Command aimAtTarget(PhotonCamera camera) {

    return run(
        () -> {
          PhotonPipelineResult result = camera.getLatestResult();
          if (result.hasTargets()) {
            drive(
                getTargetSpeeds(
                    0,
                    0,
                    Rotation2d.fromDegrees(
                        result
                            .getBestTarget()
                            .getYaw()))); // Not sure if this will work, more math may be required.
          }
        });
  }
}