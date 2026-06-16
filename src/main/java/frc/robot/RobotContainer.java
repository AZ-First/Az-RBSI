// Copyright (c) 2024-2026 Az-FIRST
// http://github.com/AZ-First
// Copyright (c) 2021-2026 Littleton Robotics
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

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.CANBuses;
import frc.robot.Constants.Cameras;
import frc.robot.Constants.OperatorConstants;
import frc.robot.FieldConstants.AprilTagLayoutType;
import frc.robot.commands.AutopilotCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.accelerometer.Accelerometer;
import frc.robot.subsystems.accelerometer.RioAccelIO;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveOdometry;
import frc.robot.subsystems.drive.SwerveConstants;
import frc.robot.subsystems.flywheel_example.Flywheel;
import frc.robot.subsystems.flywheel_example.FlywheelIO;
import frc.robot.subsystems.flywheel_example.FlywheelIOSim;
import frc.robot.subsystems.imu.Imu;
import frc.robot.subsystems.imu.ImuIO;
import frc.robot.subsystems.imu.ImuIOSim;
import frc.robot.subsystems.vision.CameraSweepEvaluator;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.OverrideSwitches;
import frc.robot.util.RBSICANBusRegistry;
import frc.robot.util.RBSICANHealth;
import frc.robot.util.RBSIController;
import frc.robot.util.RBSIEnum.AutoType;
import frc.robot.util.RBSIEnum.DriveStyle;
import frc.robot.util.RBSIEnum.Mode;
import frc.robot.util.RBSIPowerMonitor;
import java.util.Arrays;
import java.util.List;
import java.util.Set;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;

/** This is the location for defining robot hardware, commands, and controller button bindings. */
public class RobotContainer {

  /** Define the Driver and, optionally, the Operator/Co-Driver Controllers */
  final RBSIController driverController = RBSIController.createDriverController(0); // Main Driver

  final RBSIController operatorController =
      RBSIController.createDriverController(1); // Second Operator

  final OverrideSwitches overrides = new OverrideSwitches(2); // Console toggle switches

  // These two are needed for the Sweep evaluator for camera FOV simulation
  final CommandJoystick joystick3 = new CommandJoystick(3); //  Joystick for CamersSweepEvaluator
  private final CameraSweepEvaluator sweep;

  /** Declare the robot subsystems here ************************************ */
  // These are the "Active Subsystems" that the robot controls
  private final Drive m_drivebase;

  private final Flywheel m_flywheel;

  // ... Add additional subsystems here (e.g., elevator, arm, etc.)

  // These are "Virtual Subsystems" that report information but have no motors
  private final Imu m_imu;
  private final Vision m_vision;

  @SuppressWarnings("unused")
  private final DriveOdometry m_driveOdometry;

  @SuppressWarnings("unused")
  private final Accelerometer m_accel;

  @SuppressWarnings("unused")
  private final RBSIPowerMonitor m_power;

  @SuppressWarnings("unused")
  private List<RBSICANHealth> canHealth;

  /** Dashboard inputs ***************************************************** */
  // AutoChoosers for both supported path planning types
  private final LoggedDashboardChooser<Command> autoChooserPathPlanner;

  private final LoggedDashboardChooser<Command> autoChooserChoreo;
  private final AutoFactory autoFactoryChoreo;

  private final LoggedDashboardChooser<DriveStyle> driveStyle =
      new LoggedDashboardChooser<>("Drive Style");

  // Input estimated battery capacity (if full, use printed value)
  private final LoggedTunableNumber batteryCapacity =
      new LoggedTunableNumber("Battery Amp-Hours", 18.0);
  // EXAMPLE TUNABLE FLYWHEEL SPEED INPUT FROM DASHBOARD
  private final LoggedTunableNumber flywheelSpeedInput =
      new LoggedTunableNumber("Flywheel Speed", 1500.0);

  // Alerts
  private final Alert aprilTagLayoutAlert = new Alert("", AlertType.INFO);

  /**
   * Constructor for the Robot Container. This container holds subsystems, opertator interface
   * devices, and commands.
   */
  public RobotContainer() {

    // Instantiate Robot Subsystems based on RobotType
    switch (Constants.getMode()) {
      case REAL:
        // Real robot, instantiate hardware IO implementations

        // Register the CANBus
        RBSICANBusRegistry.initReal(CANBuses.RIO, CANBuses.DRIVE);

        // YAGSL drivebase, get config from deploy directory
        // Get the IMU instance
        m_imu = new Imu(SwerveConstants.kImu.factory.get());

        m_drivebase = new Drive(m_imu);
        m_driveOdometry = new DriveOdometry(m_drivebase, m_imu, m_drivebase.getModules());
        m_vision =
            new Vision(
                m_drivebase, m_drivebase::addVisionMeasurement, buildVisionIOsReal(m_drivebase));
        m_flywheel = new Flywheel(new FlywheelIOSim()); // new Flywheel(new FlywheelIOTalonFX());
        m_accel = new Accelerometer(m_imu);
        sweep = null;
        break;

      case SIM:
        RBSICANBusRegistry.initSim(CANBuses.RIO, CANBuses.DRIVE);

        m_imu = new Imu(new ImuIOSim());
        m_drivebase = new Drive(m_imu);
        m_driveOdometry = new DriveOdometry(m_drivebase, m_imu, m_drivebase.getModules());
        m_vision =
            new Vision(
                m_drivebase, m_drivebase::addVisionMeasurement, buildVisionIOsSim(m_drivebase));
        m_flywheel = new Flywheel(new FlywheelIOSim());
        m_accel = new Accelerometer(m_imu);

        // CameraSweepEvaluator uses isolated sweep-only Photon cameras to avoid colliding with
        // robot vision camera names in NetworkTables.
        if (Constants.getVisionType() == frc.robot.util.RBSIEnum.VisionType.PHOTON
            && Cameras.ALL.length >= 2) {
          VisionSystemSim visionSim = new VisionSystemSim("CameraSweepWorld");
          visionSim.addAprilTags(FieldConstants.aprilTagLayout);
          PhotonCameraSim[] simCams = new PhotonCameraSim[Cameras.ALL.length];
          for (int i = 0; i < Cameras.ALL.length; i++) {
            var cfg = Cameras.ALL[i];
            PhotonCamera photonCam = new PhotonCamera("Sweep_" + cfg.name());
            PhotonCameraSim camSim = new PhotonCameraSim(photonCam, cfg.simProps());
            visionSim.addCamera(camSim, cfg.robotToCamera());
            simCams[i] = camSim;
          }
          sweep = new CameraSweepEvaluator(visionSim, simCams[0], simCams[1]);
        } else {
          sweep = null;
        }

        break;

      default:
        // Replayed robot, disable IO implementations
        RBSICANBusRegistry.initSim(CANBuses.RIO, CANBuses.DRIVE);
        m_imu = new Imu(new ImuIO() {});
        m_drivebase = new Drive(m_imu);
        m_driveOdometry = new DriveOdometry(m_drivebase, m_imu, m_drivebase.getModules());
        m_vision =
            new Vision(
                m_drivebase, m_drivebase::addVisionMeasurement, buildVisionIOsReplay(m_drivebase));

        m_flywheel = new Flywheel(new FlywheelIO() {});
        m_accel = new Accelerometer(m_imu, RioAccelIO.noop());
        sweep = null;
        break;
    }

    // Init all CAN busses specified in the `Constants.CANBuses` class
    canHealth = Arrays.stream(Constants.CANBuses.ALL).map(RBSICANHealth::new).toList();

    // In addition to the initial battery capacity from the Dashbaord, ``RBSIPowerMonitor`` takes
    // all the non-drivebase subsystems for which you wish to have power monitoring; DO NOT
    // include ``m_drivebase``, as that is automatically monitored.
    m_power = new RBSIPowerMonitor(batteryCapacity, m_flywheel);

    // Define PathPlanner named commands before any autos or paths are created.
    defineAutoCommands();

    // Set up the SmartDashboard Auto Chooser based on auto type
    switch (Constants.getAutoType()) {
      case MANUAL:
        // This is where the "Leave Auto" will go
        // ...
        // Set the others to null
        autoChooserPathPlanner = null;
        autoChooserChoreo = null;
        autoFactoryChoreo = null;
        break;

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
                m_drivebase::resetPose, // A function that resets the current robot pose to the
                // provided Pose2d
                m_drivebase::followTrajectory, // The drive subsystem trajectory follower
                true, // If alliance flipping should be enabled
                m_drivebase // The drive subsystem
                );
        autoChooserChoreo = new LoggedDashboardChooser<>("Choreo Auto Choices");
        autoChooserChoreo.addDefaultOption("Nothing", Commands.none());
        autoChooserChoreo.addOption("twoPieceAuto", twoPieceAuto().cmd());
        // Set the others to null
        autoChooserPathPlanner = null;
        break;

      default:
        // Then, throw the error
        throw new RuntimeException(
            "Incorrect AUTO type selected in Constants: " + Constants.getAutoType());
    }

    // Get drive style from the Dashboard Chooser. The constant controls the boot default, and the
    // dashboard chooser lets teams swap stick layouts between drivers without recompiling.
    driveStyle.addDefaultOption(
        OperatorConstants.kDriveStyle.name(), OperatorConstants.kDriveStyle);
    for (DriveStyle style : DriveStyle.values()) {
      if (style != OperatorConstants.kDriveStyle) {
        driveStyle.addOption(style.name(), style);
      }
    }

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

    // SET STANDARD DRIVING AS DEFAULT COMMAND FOR THE DRIVEBASE
    m_drivebase.setDefaultCommand(
        DriveCommands.fieldRelativeDrive(
            m_drivebase, () -> -getDriveStickY(), () -> -getDriveStickX(), () -> -getTurnStickX()));

    // ** Example Commands -- Remap, remove, or change as desired **
    // Press B / Circle button while driving --> ROBOT-CENTRIC
    driverController
        .robotRelative()
        .whileTrue(
            DriveCommands.robotRelativeDrive(
                m_drivebase,
                () -> -getDriveStickY(),
                () -> -getDriveStickX(),
                () -> -getTurnStickX()));

    // Press A / Cross button -> BRAKE
    driverController.brake().onTrue(DriveCommands.setBrakeMode(m_drivebase, true));

    // Press X / Square button --> Stop with wheels in X-Lock position
    driverController.xLock().whileTrue(DriveCommands.stopWithX(m_drivebase));

    // Press Y / Triangle button --> Manually Re-Zero the Gyro
    driverController.zeroGyro().onTrue(DriveCommands.zeroHeadingForAlliance(m_drivebase));

    // Press RIGHT BUMPER / R1 --> Run the example flywheel
    driverController
        .runFlywheel()
        .whileTrue(
            Commands.startEnd(
                () -> m_flywheel.runVelocity(flywheelSpeedInput.get()),
                m_flywheel::stop,
                m_flywheel));

    // Press LEFT BUMPER / L1 --> Drive to a demo pose offset defined in OperatorConstants
    driverController
        .autopilotDemo()
        .whileTrue(
            Commands.defer(
                () -> {
                  // Demo target relative to the current pose.
                  Pose2d pose =
                      m_drivebase
                          .getPose()
                          .transformBy(
                              new Transform2d(
                                  OperatorConstants.kAutopilotDemoXOffsetMeters,
                                  0.0,
                                  Rotation2d.kZero));

                  // Alternatively, you could define a pose in a separate module and call it here.
                  //
                  // Example from 2025 Reefscape:
                  // --------
                  // pose = ReefPoses.kBluePoleE;

                  return AutopilotCommands.runAutopilot(m_drivebase, pose);
                },
                Set.of(m_drivebase)));

    // Press POV LEFT to nudge the robot left
    driverController
        .povLeft()
        .whileTrue(
            DriveCommands.robotRelativeNudge(
                m_drivebase, 0.0, OperatorConstants.kRobotRelativeNudgeSpeedMetersPerSec, 0.0));
    driverController
        .povRight()
        .whileTrue(
            DriveCommands.robotRelativeNudge(
                m_drivebase, 0.0, -OperatorConstants.kRobotRelativeNudgeSpeedMetersPerSec, 0.0));
    driverController
        .povUp()
        .whileTrue(
            DriveCommands.robotRelativeNudge(
                m_drivebase, OperatorConstants.kRobotRelativeNudgeSpeedMetersPerSec, 0.0, 0.0));
    driverController
        .povDown()
        .whileTrue(
            DriveCommands.robotRelativeNudge(
                m_drivebase, -OperatorConstants.kRobotRelativeNudgeSpeedMetersPerSec, 0.0, 0.0));

    if (Constants.getMode() == Mode.SIM) {
      // IN SIMULATION ONLY:
      // Double-press the A button on Joystick3 to run the CameraSweepEvaluator
      // Use WPILib's built-in double-press binding
      joystick3
          .button(1)
          .multiPress(2, 0.2)
          .onTrue(
              Commands.runOnce(
                  () -> {
                    try {
                      sweep.runFullSweep(
                          Filesystem.getOperatingDirectory()
                              .toPath()
                              .resolve("camera_sweep.csv")
                              .toString());
                    } catch (Exception e) {
                      DriverStation.reportError("Camera sweep failed", e.getStackTrace());
                    }
                  }));
    }
  }

  /**
   * Use this to pass the MANUAL SHOOT FUEL command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getManualAuto() {
    // NOTE:
    //
    // For teams not using PathPlanner, this auto may be used to simply shoot the pre-loaded fuel
    // into the HUB during AUTO.  Since shooters are beyond the scope of Az-RBSI, you will have to
    // write your own command and call it here.

    // Replace Commands.none() with your command that shoots fuel into the HUB.
    return Commands.none();
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
  public Command getAutonomousCommandChoreo() {
    return autoChooserChoreo.get();
  }

  /** Updates the alerts. */
  public void updateAlerts() {
    // AprilTag layout alert
    boolean aprilTagAlertActive =
        Constants.getAprilTagLayoutType() != AprilTagLayoutType.REBUILT_WELDED;
    aprilTagLayoutAlert.set(aprilTagAlertActive);
    if (aprilTagAlertActive) {
      aprilTagLayoutAlert.setText(
          "Non-official AprilTag layout in use ("
              + Constants.getAprilTagLayoutType().toString()
              + ").");
    }
  }

  /** Drivetrain getter method for use with Robot.java */
  public Drive getDrivebase() {
    return m_drivebase;
  }

  /** Vision getter method for use with Robot.java */
  public Vision getVision() {
    return m_vision;
  }

  /**
   * Set up the SysID routines from AdvantageKit
   *
   * <p>NOTE: These are currently only accessible with Constants.AutoType.PATHPLANNER
   */
  private void definesysIdRoutines() {
    if (Constants.getAutoType() == AutoType.PATHPLANNER) {
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
          "Flywheel SysId Voltage (Quasistatic Forward)",
          m_flywheel.sysIdVoltageQuasistatic(SysIdRoutine.Direction.kForward));
      autoChooserPathPlanner.addOption(
          "Flywheel SysId Voltage (Quasistatic Reverse)",
          m_flywheel.sysIdVoltageQuasistatic(SysIdRoutine.Direction.kReverse));
      autoChooserPathPlanner.addOption(
          "Flywheel SysId Voltage (Dynamic Forward)",
          m_flywheel.sysIdVoltageDynamic(SysIdRoutine.Direction.kForward));
      autoChooserPathPlanner.addOption(
          "Flywheel SysId Voltage (Dynamic Reverse)",
          m_flywheel.sysIdVoltageDynamic(SysIdRoutine.Direction.kReverse));
      autoChooserPathPlanner.addOption(
          "Flywheel SysId Duty Cycle (Quasistatic Forward)",
          m_flywheel.sysIdDutyCycleQuasistatic(SysIdRoutine.Direction.kForward));
      autoChooserPathPlanner.addOption(
          "Flywheel SysId Duty Cycle (Quasistatic Reverse)",
          m_flywheel.sysIdDutyCycleQuasistatic(SysIdRoutine.Direction.kReverse));
      autoChooserPathPlanner.addOption(
          "Flywheel SysId Duty Cycle (Dynamic Forward)",
          m_flywheel.sysIdDutyCycleDynamic(SysIdRoutine.Direction.kForward));
      autoChooserPathPlanner.addOption(
          "Flywheel SysId Duty Cycle (Dynamic Reverse)",
          m_flywheel.sysIdDutyCycleDynamic(SysIdRoutine.Direction.kReverse));
    }
  }

  // Vision Factories
  // Vision Factories (REAL)
  private VisionIO[] buildVisionIOsReal(Drive drive) {
    return switch (Constants.getVisionType()) {
      case PHOTON ->
          Arrays.stream(Constants.Cameras.ALL)
              .map(c -> (VisionIO) new VisionIOPhotonVision(c.name(), c.robotToCamera()))
              .toArray(VisionIO[]::new);

      case LIMELIGHT ->
          Arrays.stream(Constants.Cameras.ALL)
              .map(c -> (VisionIO) new VisionIOLimelight(c.name(), drive::getHeading))
              .toArray(VisionIO[]::new);

      case NONE -> new VisionIO[] {}; // recommended: no cameras
    };
  }

  // Vision Factories (SIM)
  private VisionIO[] buildVisionIOsSim(Drive drive) {
    return switch (Constants.getVisionType()) {
      case PHOTON ->
          Arrays.stream(Constants.Cameras.ALL)
              .map(
                  c ->
                      (VisionIO)
                          new VisionIOPhotonVisionSim(
                              c.name(), c.robotToCamera(), c.simProps(), drive::getPose))
              .toArray(VisionIO[]::new);

      case LIMELIGHT ->
          Arrays.stream(Constants.Cameras.ALL)
              .map(c -> (VisionIO) new VisionIOLimelight(c.name(), drive::getHeading))
              .toArray(VisionIO[]::new);

      case NONE -> new VisionIO[] {};
    };
  }

  // Vision Factories (REPLAY)
  private VisionIO[] buildVisionIOsReplay(Drive drive) {
    var cams = Constants.Cameras.ALL;

    VisionIO[] ios = new VisionIO[cams.length];
    for (int i = 0; i < cams.length; i++) {
      ios[i] =
          new VisionIO() {
            @Override
            public void updateInputs(VisionIOInputs inputs) {
              // Intentionally empty.
              // Logger.processInputs("Vision/Camera" + i, inputs) will populate these from the log.
            }
          };
    }
    return ios;
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

  private DriveStyle getSelectedDriveStyle() {
    DriveStyle selected = driveStyle.get();
    return selected != null ? selected : OperatorConstants.kDriveStyle;
  }

  private double getDriveStickY() {
    return switch (getSelectedDriveStyle()) {
      case GAMER -> driverController.getRightY();
      case TANK -> driverController.getLeftY();
    };
  }

  private double getDriveStickX() {
    return switch (getSelectedDriveStyle()) {
      case GAMER -> driverController.getRightX();
      case TANK -> driverController.getLeftX();
    };
  }

  private double getTurnStickX() {
    return switch (getSelectedDriveStyle()) {
      case GAMER -> driverController.getLeftX();
      case TANK -> driverController.getRightX();
    };
  }
}
