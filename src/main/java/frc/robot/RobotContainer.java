// Copyright (c) 2024-2025 Az-FIRST
// http://github.com/AZ-First
// Copyright (c) 2021-2025 FRC 6328
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

import static frc.robot.Constants.Cameras.*;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.AprilTagConstants.AprilTagLayoutType;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.accelerometer.Accelerometer;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.flywheel_example.Flywheel;
import frc.robot.subsystems.flywheel_example.FlywheelIO;
import frc.robot.subsystems.flywheel_example.FlywheelIOSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;
import frc.robot.util.GetJoystickValue;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.OverrideSwitches;
import frc.robot.util.PowerMonitoring;
import frc.robot.util.RBSIEnum;
import java.util.List;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/** This is the location for defining robot hardware, commands, and controller button bindings. */
public class RobotContainer {

  private static final boolean USE_MAPLESIM = true;
  public static final boolean MAPLESIM = USE_MAPLESIM && Robot.isSimulation();

  // **** This is a Pathplanner On-the-Fly Command ****/
  // Create a list of waypoints from poses. Each pose represents one waypoint.
  // The rotation component of the pose should be the direction of travel. Do not use
  // holonomic rotation.
  List<Waypoint> woahpoints =
      PathPlannerPath.waypointsFromPoses(
          new Pose2d(8.180, 6.184, Rotation2d.fromDegrees(0)),
          new Pose2d(9.4, 6.184, Rotation2d.fromDegrees(0)));

  PathConstraints constraints =
      new PathConstraints(1.0, 1.0, 2 * Math.PI, 4 * Math.PI); // The constraints for this path.
  // PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0); // You can
  // also use unlimited constraints, only limited by motor torque and nominal battery
  // voltage

  // Create the path using the waypoints created above
  PathPlannerPath woah =
      new PathPlannerPath(
          woahpoints,
          constraints,
          null, // The ideal starting state, this is only relevant for pre-planned paths,
          // so
          // can be null for on-the-fly paths.
          new GoalEndState(
              0.0,
              Rotation2d.fromDegrees(
                  180)) // Goal end state. You can set a holonomic rotation here. If
          // using a
          // differential drivetrain, the rotation will have no effect.
          );

  // Prevent the path from being flipped if the coordinates are already correct

  /** Define the Driver and, optionally, the Operator/Co-Driver Controllers */
  // Replace with ``CommandPS4Controller`` or ``CommandJoystick`` if needed
  final CommandXboxController driverController = new CommandXboxController(0); // Main Driver

  final CommandXboxController operatorController = new CommandXboxController(1); // Second Operator
  final OverrideSwitches overrides = new OverrideSwitches(2); // Console toggle switches

  /** Declare the robot subsystems here ************************************ */
  // These are the "Active Subsystems" that the robot controlls
  public final Drive m_drivebase;

  private final Flywheel m_flywheel;
  // These are "Virtual Subsystems" that report information but have no motors
  private final Accelerometer m_accel;
  private final Vision m_vision;
  private final PowerMonitoring m_power;

  /** Dashboard inputs ***************************************************** */
  // AutoChoosers for both supported path planning types
  private final LoggedDashboardChooser<Command> autoChooserPathPlanner;

  private final AutoChooser autoChooserChoreo;
  private final AutoFactory autoFactoryChoreo;
  // Input estimated battery capacity (if full, use printed value)
  private final LoggedTunableNumber batteryCapacity =
      new LoggedTunableNumber("Battery Amp-Hours", 18.0);
  // EXAMPLE TUNABLE FLYWHEEL SPEED INPUT FROM DASHBOARD
  private final LoggedTunableNumber flywheelSpeedInput =
      new LoggedTunableNumber("Flywheel Speed", 1500.0);

  // Alerts
  private final Alert aprilTagLayoutAlert = new Alert("", AlertType.INFO);

  public static RobotContainer instance;

  /**
   * Constructor for the Robot Container. This container holds subsystems, opertator interface
   * devices, and commands.
   */
  public RobotContainer() {
    instance = this;
    woah.preventFlipping = true;
    // Instantiate Robot Subsystems based on RobotType
    switch (Constants.getMode()) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // YAGSL drivebase, get config from deploy directory
        m_drivebase = new Drive();
        m_flywheel = new Flywheel(new FlywheelIOSim()); // new Flywheel(new FlywheelIOTalonFX());
        m_vision =
            switch (Constants.getVisionType()) {
              case PHOTON ->
                  new Vision(
                      m_drivebase::addVisionMeasurement,
                      new VisionIOPhotonVision(camera0Name, robotToCamera0),
                      new VisionIOPhotonVision(camera1Name, robotToCamera1));
              case LIMELIGHT ->
                  new Vision(
                      m_drivebase::addVisionMeasurement,
                      new VisionIOLimelight(camera0Name, m_drivebase::getRotation),
                      new VisionIOLimelight(camera1Name, m_drivebase::getRotation));
              case NONE ->
                  new Vision(
                      m_drivebase::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
              default -> null;
            };
        m_accel = new Accelerometer(m_drivebase.getGyro());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        m_drivebase = new Drive();
        m_flywheel = new Flywheel(new FlywheelIOSim() {});
        m_vision =
            new Vision(
                m_drivebase::addVisionMeasurement,
                new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, m_drivebase::getPose),
                new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, m_drivebase::getPose));
        m_accel = new Accelerometer(m_drivebase.getGyro());
        break;

      default:
        // Replayed robot, disable IO implementations
        m_drivebase = new Drive();
        m_flywheel = new Flywheel(new FlywheelIO() {});
        m_vision =
            new Vision(m_drivebase::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        m_accel = new Accelerometer(m_drivebase.getGyro());
        break;
    }

    // In addition to the initial battery capacity from the Dashbaord, ``PowerMonitoring`` takes all
    // the non-drivebase subsystems for which you wish to have power monitoring; DO NOT include
    // ``m_drivebase``, as that is automatically monitored.
    m_power = new PowerMonitoring(batteryCapacity, m_flywheel);

    // Set up the SmartDashboard Auto Chooser based on auto type
    switch (Constants.getAutoType()) {
      case PATHPLANNER:
        autoChooserPathPlanner =
            new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
        // Set the others to null
        autoChooserChoreo = null;
        autoFactoryChoreo = null;
        break;

      case CHOREO:
        autoFactoryChoreo =
            new AutoFactory(
                m_drivebase::getPose, // A function that returns the current robot pose
                m_drivebase::resetOdometry, // A function that resets the current robot pose to the
                // provided Pose2d
                m_drivebase::followTrajectory, // The drive subsystem trajectory follower
                true, // If alliance flipping should be enabled
                m_drivebase // The drive subsystem
                );
        autoChooserChoreo = new AutoChooser();
        autoChooserChoreo.addRoutine("twoPieceAuto", this::twoPieceAuto);
        // Set the others to null
        autoChooserPathPlanner = null;
        break;

      default:
        // Then, throw the error
        throw new RuntimeException(
            "Incorrect AUTO type selected in Constants: " + Constants.getAutoType());
    }

    // Define Auto commands
    defineAutoCommands();
    // Define SysIs Routines
    definesysIdRoutines();
    // Configure the button and trigger bindings
    configureBindings();
  }

  /** Use this method to define your Autonomous commands for use with PathPlanner / Choreo */
  private void defineAutoCommands() {

    // NamedCommands.registerCommand("Zero", Commands.runOnce(() -> m_drivebase.zero()));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureBindings() {

    // Send the proper joystick input based on driver preference -- Set this in `Constants.java`
    GetJoystickValue driveStickY;
    GetJoystickValue driveStickX;
    GetJoystickValue turnStickX;
    if (OperatorConstants.kDriveLeftTurnRight) {
      driveStickY = driverController::getLeftY;
      driveStickX = driverController::getLeftX;
      turnStickX = driverController::getRightX;
    } else {
      driveStickY = driverController::getRightY;
      driveStickX = driverController::getRightX;
      turnStickX = driverController::getLeftX;
    }

    // SET STANDARD DRIVING AS DEFAULT COMMAND FOR THE DRIVEBASE
    m_drivebase.setDefaultCommand(
        DriveCommands.fieldRelativeDrive(
            m_drivebase,
            () -> -driveStickY.value(),
            () -> -driveStickX.value(),
            () -> -turnStickX.value()));

    // ** Example Commands -- Remap, remove, or change as desired **
    // Press B button while driving --> ROBOT-CENTRIC
    driverController
        .b()
        .onTrue(
            Commands.runOnce(
                () ->
                    DriveCommands.robotRelativeDrive(
                        m_drivebase,
                        () -> -driveStickY.value(),
                        () -> -driveStickX.value(),
                        () -> turnStickX.value()),
                m_drivebase));

    // Press A button -> BRAKE
    driverController
        .a()
        .whileTrue(Commands.runOnce(() -> m_drivebase.setMotorBrake(true), m_drivebase));

    // Press X button --> Stop with wheels in X-Lock position
    driverController.x().onTrue(Commands.runOnce(m_drivebase::stopWithX, m_drivebase));

    // Press Y button --> Manually Re-Zero the Gyro
    driverController
        .y()
        .onTrue(
            Commands.runOnce(
                    () ->
                        m_drivebase.resetPose(
                            new Pose2d(m_drivebase.getPose().getTranslation(), new Rotation2d())),
                    m_drivebase)
                .ignoringDisable(true));

    // Press RIGHT BUMPER --> Run the example flywheel
    driverController
        .rightBumper()
        .whileTrue(
            Commands.startEnd(
                () -> m_flywheel.runVelocity(flywheelSpeedInput.get()),
                m_flywheel::stop,
                m_flywheel));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommandPathPlanner() {
    // Use the ``autoChooser`` to define your auto path from the SmartDashboard
    return autoChooserPathPlanner.get();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public void getAutonomousCommandChoreo() {
    // Put the auto chooser on the dashboard
    SmartDashboard.putData(autoChooserChoreo);

    // Schedule the selected auto during the autonomous period
    RobotModeTriggers.autonomous().whileTrue(autoChooserChoreo.selectedCommandScheduler());
  }

  /** Set the motor neutral mode to BRAKE / COAST for T/F */
  public void setMotorBrake(boolean brake) {
    m_drivebase.setMotorBrake(brake);
  }

  /** Updates the alerts. */
  public void updateAlerts() {
    // AprilTag layout alert
    boolean aprilTagAlertActive = Constants.getAprilTagLayoutType() != AprilTagLayoutType.OFFICIAL;
    aprilTagLayoutAlert.set(aprilTagAlertActive);
    if (aprilTagAlertActive) {
      aprilTagLayoutAlert.setText(
          "Non-official AprilTag layout in use ("
              + Constants.getAprilTagLayoutType().toString()
              + ").");
    }
  }

  /**
   * Set up the SysID routines from AdvantageKit
   *
   * <p>NOTE: These are currently only accessible with Constants.AutoType.PATHPLANNER
   */
  private void definesysIdRoutines() {
    if (Constants.getAutoType() == RBSIEnum.AutoType.PATHPLANNER) {
      // Drivebase characterization
      autoChooserPathPlanner.addOption(
          "Drive Wheel Radius Characterization",
          DriveCommands.wheelRadiusCharacterization(m_drivebase));
      autoChooserPathPlanner.addOption(
          "Drive Simple FF Characterization",
          DriveCommands.feedforwardCharacterization(m_drivebase));
      autoChooserPathPlanner.addOption(
          "Drive SysId (Quasistatic Forward)",
          m_drivebase.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
      autoChooserPathPlanner.addOption(
          "Drive SysId (Quasistatic Reverse)",
          m_drivebase.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
      autoChooserPathPlanner.addOption(
          "Drive SysId (Dynamic Forward)",
          m_drivebase.sysIdDynamic(SysIdRoutine.Direction.kForward));
      autoChooserPathPlanner.addOption(
          "Drive SysId (Dynamic Reverse)",
          m_drivebase.sysIdDynamic(SysIdRoutine.Direction.kReverse));

      // Example Flywheel SysId Characterization
      autoChooserPathPlanner.addOption(
          "Flywheel SysId (Quasistatic Forward)",
          m_flywheel.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
      autoChooserPathPlanner.addOption(
          "Flywheel SysId (Quasistatic Reverse)",
          m_flywheel.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
      autoChooserPathPlanner.addOption(
          "Flywheel SysId (Dynamic Forward)",
          m_flywheel.sysIdDynamic(SysIdRoutine.Direction.kForward));
      autoChooserPathPlanner.addOption(
          "Flywheel SysId (Dynamic Reverse)",
          m_flywheel.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    }
  }

  /**
   * Example Choreo auto command
   *
   * <p>NOTE: This would normally be in a spearate file.
   */
  private AutoRoutine twoPieceAuto() {
    AutoRoutine routine = autoFactoryChoreo.newRoutine("twoPieceAuto");

    // Load the routine's trajectories
    AutoTrajectory pickupTraj = routine.trajectory("pickupGamepiece");
    AutoTrajectory scoreTraj = routine.trajectory("scoreGamepiece");

    // When the routine begins, reset odometry and start the first trajectory
    routine.active().onTrue(Commands.sequence(pickupTraj.resetOdometry(), pickupTraj.cmd()));

    // Starting at the event marker named "intake", run the intake
    // pickupTraj.atTime("intake").onTrue(intakeSubsystem.intake());

    // When the trajectory is done, start the next trajectory
    pickupTraj.done().onTrue(scoreTraj.cmd());

    // While the trajectory is active, prepare the scoring subsystem
    // scoreTraj.active().whileTrue(scoringSubsystem.getReady());

    // When the trajectory is done, score
    // scoreTraj.done().onTrue(scoringSubsystem.score());

    return routine;
  }
}
