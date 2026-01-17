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

package frc.robot;

import com.revrobotics.util.StatusLogger;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.VirtualSubsystem;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.urcl.URCL;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private Command m_autoCommandPathPlanner;
  private RobotContainer m_robotContainer;
  private Timer m_disabledTimer;

  // Define simulation fields here
  private VisionSystemSim visionSim;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    // Record metadata
    Logger.recordMetadata("Robot", Constants.getRobot().toString());
    Logger.recordMetadata("TuningMode", Boolean.toString(Constants.tuningMode));
    Logger.recordMetadata("RuntimeType", getRuntimeType().toString());
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    Logger.recordMetadata(
        "GitDirty",
        switch (BuildConstants.DIRTY) {
          case 0 -> "All changes committed";
          case 1 -> "Uncommitted changes";
          default -> "Unknown";
        });

    // Set up data receivers & replay source
    switch (Constants.getMode()) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // Initialize URCL
    Logger.registerURCL(URCL.startExternal());
    StatusLogger.disableAutoLogging(); // Disable REVLib's built-in logging
    // LoggedPowerDistribution.getInstance(PowerConstants.kPDMCANid, PowerConstants.kPDMType);

    // Start AdvantageKit logger
    Logger.start();

    // Instantiate our RobotContainer. This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    // Create a timer to disable motor brake a few seconds after disable. This will let the robot
    // stop immediately when disabled, but then also let it be pushed more
    m_disabledTimer = new Timer();
  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {
    // Switch thread to high priority to improve loop timing
    if (isReal()) {
      Threads.setCurrentThreadPriority(true, 99);
    }

    // Run all virtual subsystems each time through the loop
    VirtualSubsystem.periodicAll();

    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled commands, running already-scheduled commands, removing
    // finished or interrupted commands, and running subsystem periodic() methods.
    // This must be called from the robot's periodic block in order for anything in
    // the Command-based framework to work.
    CommandScheduler.getInstance().run();

    // Return to normal thread priority
    Threads.setCurrentThreadPriority(false, 10);
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    // Set the brakes to stop robot motion
    m_robotContainer.setMotorBrake(true);
    m_disabledTimer.reset();
    m_disabledTimer.start();
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    // After WHEEL_LOCK_TIME has elapsed, release the drive brakes
    if (m_disabledTimer.hasElapsed(Constants.DrivebaseConstants.kWheelLockTime)) {
      m_robotContainer.setMotorBrake(false);
      m_disabledTimer.stop();
    }
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {

    // Just in case, cancel all running commands
    CommandScheduler.getInstance().cancelAll();
    m_robotContainer.setMotorBrake(true);

    // TODO: Make sure Gyro inits here with whatever is in the path planning thingie
    switch (Constants.getAutoType()) {
      case MANUAL:
        CommandScheduler.getInstance().schedule(m_robotContainer.getManualAuto());
        break;

      case PATHPLANNER:
        m_autoCommandPathPlanner = m_robotContainer.getAutonomousCommandPathPlanner();
        // schedule the autonomous command
        if (m_autoCommandPathPlanner != null) {
          CommandScheduler.getInstance().schedule(m_autoCommandPathPlanner);
        }
        break;

      case CHOREO:
        m_robotContainer.getAutonomousCommandChoreo();
        break;
      default:
        throw new RuntimeException(
            "Incorrect AUTO type selected in Constants: " + Constants.getAutoType());
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autoCommandPathPlanner != null) {
      m_autoCommandPathPlanner.cancel();
    } else {
      CommandScheduler.getInstance().cancelAll();
    }
    m_robotContainer.setMotorBrake(true);

    // In case this got set in sequential practice sessions or whatever
    FieldState.wonAuto = null;
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    // For 2026 - REBUILT, the alliance will be provided as a single character
    //   representing the color of the alliance whose goal will go inactive
    //   first (i.e. 'R' = red, 'B' = blue). This alliance's goal will be
    //   active in Shifts 2 and 4.
    //
    // https://docs.wpilib.org/en/stable/docs/yearly-overview/2026-game-data.html
    if (FieldState.wonAuto == null) {
      // Only call this code block if the signal from FMS has not yet arrived
      String gameData = DriverStation.getGameSpecificMessage();
      if (gameData.length() > 0) {
        switch (gameData.charAt(0)) {
          case 'B':
            // Blue case code
            FieldState.wonAuto = DriverStation.Alliance.Blue;
            break;
          case 'R':
            // Red case code
            FieldState.wonAuto = DriverStation.Alliance.Red;
            break;
          default:
            // This is corrupt data, do nothing
            break;
        }
      }
    }
    // Anything else for the teleopPeriodic() function

  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
    visionSim = new VisionSystemSim("main");

    // Load AprilTag field layout
    AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    // Register AprilTags with vision simulation
    visionSim.addAprilTags(fieldLayout);

    // Simulated Camera Properties
    SimCameraProperties cameraProp = new SimCameraProperties();
    // A 1280 x 800 camera with a 100 degree diagonal FOV.
    cameraProp.setCalibration(1280, 800, Rotation2d.fromDegrees(100));
    // Approximate detection noise with average and standard deviation error in pixels.
    cameraProp.setCalibError(0.25, 0.08);
    // Set the camera image capture framerate (Note: this is limited by robot loop rate).
    cameraProp.setFPS(20);
    // The average and standard deviation in milliseconds of image data latency.
    cameraProp.setAvgLatencyMs(35);
    cameraProp.setLatencyStdDevMs(5);

    // Define Cameras and add to simulation
    PhotonCamera camera0 = new PhotonCamera("frontCam");
    PhotonCamera camera1 = new PhotonCamera("backCam");
    visionSim.addCamera(new PhotonCameraSim(camera0, cameraProp), Constants.Cameras.robotToCamera0);
    visionSim.addCamera(new PhotonCameraSim(camera1, cameraProp), Constants.Cameras.robotToCamera1);
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    // Update sim each sim tick
    visionSim.update(m_robotContainer.getDrivebase().getPose());
  }
}
