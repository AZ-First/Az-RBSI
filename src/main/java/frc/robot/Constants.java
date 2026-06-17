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

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.therekrab.autopilot.APConstraints;
import com.therekrab.autopilot.APProfile;
import com.therekrab.autopilot.Autopilot;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.FieldConstants.AprilTagLayoutType;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.SwerveConstants;
import frc.robot.util.Alert;
import frc.robot.util.RBSIController.Axis;
import frc.robot.util.RBSIController.Button;
import frc.robot.util.RBSIEnum.AutoType;
import frc.robot.util.RBSIEnum.CTREPro;
import frc.robot.util.RBSIEnum.DriveStyle;
import frc.robot.util.RBSIEnum.Mode;
import frc.robot.util.RBSIEnum.MotorIdleMode;
import frc.robot.util.RBSIEnum.SwerveType;
import frc.robot.util.RBSIEnum.VisionType;
import frc.robot.util.RobotDeviceId;
import java.util.Set;
import org.littletonrobotics.junction.Logger;
import org.photonvision.simulation.SimCameraProperties;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  /***************************************************************************/
  /**
   * Define the various multiple robots that use this same code (e.g., COMPBOT, DEVBOT1, SIMBOT,
   * etc.) and the operating modes of the code (REAL, SIM, or REPLAY)
   */
  private static RobotType robotType = RobotType.COMPBOT;

  // Define swerve, auto, and vision types being used
  // NOTE: Only PHOENIX6 swerve base has been tested at this point!!!
  //       If you have a swerve base with non-CTRE components, use YAGSL
  //       under strict caveat emptor -- and submit any error and bugfixes
  //       via GitHub issues.
  private static SwerveType swerveType = SwerveType.PHOENIX6; // PHOENIX6, YAGSL
  private static CTREPro phoenixPro = CTREPro.LICENSED; // LICENSED, UNLICENSED
  private static AutoType autoType = AutoType.MANUAL; // MANUAL, PATHPLANNER, CHOREO
  private static VisionType visionType = VisionType.PHOTON; // PHOTON, LIMELIGHT, NONE

  /** Enumerate the robot types (name your robots here) */
  public static enum RobotType {
    DEVBOT1, // Development / Alpha / Practice Bot
    DEVBOT2, // Development / Alpha / Practice Bot
    COMPBOT, // Competition robot
    SIMBOT // Simulated robot
  }

  /** Checks whether the correct robot is selected when deploying. */
  public static void main(String... args) {
    if (robotType == RobotType.SIMBOT) {
      System.err.println("Cannot deploy, invalid robot selected: " + robotType);
      System.exit(1);
    }
  }

  /** Disable the Hardware Abstraction Layer, if requested */
  public static boolean disableHAL = false;

  public static void disableHAL() {
    disableHAL = true;
  }

  /***************************************************************************/
  /* The remainder of this file contains physical and/or software constants for the various subsystems of the robot */

  /** General Constants **************************************************** */
  public static final double kLoopPeriodSecs = 0.02;

  public static final boolean kTuningMode = false;

  public static final double kGravityMetersPerSecSq = 9.80665;

  /************************************************************************* */
  /** Physical Constants for Robot Operation ******************************* */
  public static final class RobotConstants {

    public static final Mass kMass = Pounds.of(100.);
    public static final Matter kChassisMatter =
        new Matter(new Translation3d(0, 0, Inches.of(8).in(Meters)), kMass.in(Kilograms));
    // Robot moment of inertia; this can be obtained from a CAD model of your drivetrain. Usually,
    // this is between 3 and 8 kg*m^2.
    public static final double kMomentOfInertiaKgMetersSq = 6.8;

    // Wheel coefficient of friction
    public static final double kWheelCoefficientOfFriction = 1.2;

    // Maximum torque applied by wheel
    // Kraken X60 stall torque ~7.09 Nm; MK4i L3 gear ratio 6.12:1
    public static final double kMaxWheelTorqueNm = 43.4;

    // Insert here the orientation (CCW == +) of the Rio and IMU from the robot
    // An angle of "0." means the x-y-z markings on the device match the robot's intrinsic reference
    //   frame.
    public static final Rotation3d kRioOrientation =
        switch (getRobot()) {
          case COMPBOT -> new Rotation3d(0, 0, Units.degreesToRadians(-90));
          case DEVBOT1, DEVBOT2 -> Rotation3d.kZero;
          default -> Rotation3d.kZero;
        };
    // IMU can be one of Pigeon2 or NavX
    public static final Rotation3d kIMUOrientation =
        switch (getRobot()) {
          case COMPBOT -> Rotation3d.kZero;
          case DEVBOT1, DEVBOT2 -> Rotation3d.kZero;
          default -> Rotation3d.kZero;
        };
  }

  /************************************************************************* */
  /** Power Distribution Constants ***************************************** */
  public static final class PowerConstants {

    // Power Distribution Module Configuration
    public static final PowerDistribution.ModuleType kPdmType = PowerDistribution.ModuleType.kRev;
    public static final int kPdmCanId = 1;

    // Current Limits
    public static final double kTotalMaxCurrentAmps = 120.;
    public static final double kMotorPortMaxCurrentAmps = 40.;
    public static final double kSmallPortMaxCurrentAmps = 20.;

    // Brownout voltage levels
    public static final double kWarningVoltage = 7.5;
    public static final double kLimitingVoltage = 7.0;
    public static final double kCriticalVoltage = 6.5;
  }

  /************************************************************************* */
  /** List of Robot CAN Busses ********************************************* */
  public static final class CANBuses {
    public static final String RIO = "rio";
    public static final String DRIVE = "DriveTrain";

    public static final String[] ALL = {RIO, DRIVE};
  }

  /************************************************************************* */
  /** List of Robot Device CAN and Power Distribution Circuit IDs ********** */
  public static class RobotDevices {

    /* DRIVETRAIN CAN DEVICE IDS */
    // Input the correct Power Distribution Module port for each motor!!!!
    // NOTE: The CAN ID's are set in the Swerve Generator (Phoenix Tuner or YAGSL)

    // Front Left
    public static final RobotDeviceId FL_DRIVE =
        new RobotDeviceId(SwerveConstants.kFLDriveMotorId, SwerveConstants.kFLDriveCanbus, 18);
    public static final RobotDeviceId FL_ROTATION =
        new RobotDeviceId(SwerveConstants.kFLSteerMotorId, SwerveConstants.kFLSteerCanbus, 19);
    public static final RobotDeviceId FL_CANCODER =
        new RobotDeviceId(SwerveConstants.kFLEncoderId, SwerveConstants.kFLEncoderCanbus, null);
    // Front Right
    public static final RobotDeviceId FR_DRIVE =
        new RobotDeviceId(SwerveConstants.kFRDriveMotorId, SwerveConstants.kFRDriveCanbus, 17);
    public static final RobotDeviceId FR_ROTATION =
        new RobotDeviceId(SwerveConstants.kFRSteerMotorId, SwerveConstants.kFRSteerCanbus, 16);
    public static final RobotDeviceId FR_CANCODER =
        new RobotDeviceId(SwerveConstants.kFREncoderId, SwerveConstants.kFREncoderCanbus, null);
    // Back Left
    public static final RobotDeviceId BL_DRIVE =
        new RobotDeviceId(SwerveConstants.kBLDriveMotorId, SwerveConstants.kBLDriveCanbus, 1);
    public static final RobotDeviceId BL_ROTATION =
        new RobotDeviceId(SwerveConstants.kBLSteerMotorId, SwerveConstants.kBLSteerCanbus, 0);
    public static final RobotDeviceId BL_CANCODER =
        new RobotDeviceId(SwerveConstants.kBLEncoderId, SwerveConstants.kBLEncoderCanbus, null);
    // Back Right
    public static final RobotDeviceId BR_DRIVE =
        new RobotDeviceId(SwerveConstants.kBRDriveMotorId, SwerveConstants.kBRDriveCanbus, 2);
    public static final RobotDeviceId BR_ROTATION =
        new RobotDeviceId(SwerveConstants.kBRSteerMotorId, SwerveConstants.kBRSteerCanbus, 3);
    public static final RobotDeviceId BR_CANCODER =
        new RobotDeviceId(SwerveConstants.kBREncoderId, SwerveConstants.kBREncoderCanbus, null);
    // Pigeon
    public static final RobotDeviceId PIGEON =
        new RobotDeviceId(SwerveConstants.kPigeonId, SwerveConstants.kCANbusName, null);

    /* SUBSYSTEM CAN DEVICE IDS */
    // This is where mechanism subsystem devices are defined (Including ID, bus, and power port)
    // Example:
    public static final RobotDeviceId FLYWHEEL_LEADER = new RobotDeviceId(3, CANBuses.RIO, 8);
    public static final RobotDeviceId FLYWHEEL_FOLLOWER = new RobotDeviceId(4, CANBuses.RIO, 9);

    /* BEAM BREAK and/or LIMIT SWITCH DIO CHANNELS */
    // This is where digital I/O feedback devices are defined
    // Example:
    // public static final int ELEVATOR_BOTTOM_LIMIT = 3;

    /* LINEAR SERVO PWM CHANNELS */
    // This is where PWM-controlled devices (actuators, servos, pneumatics, etc.)
    // are defined
    // Example:
    // public static final int INTAKE_SERVO = 0;
  }

  /************************************************************************* */
  /** Sensor Constants ***************************************************** */
  public static final class SensorConstants {

    // RoboRIO accelerometer sampler rate.
    public static final double kRioAccelerometerSampleRateHz = 200.0;
  }

  /************************************************************************* */
  /** Operator Constants *************************************************** */
  public static class OperatorConstants {

    // Joystick Functions
    // Set to TANK for Drive = Left Stick, Turn = Right Stick;
    // Set to GAMER for Drive = Right Stick, Turn = Left Stick;
    // RobotContainer publishes this as the default value for the Drive Style dashboard chooser.
    public static final DriveStyle kDriveStyle = DriveStyle.TANK; // TANK, GAMER

    // Joystick Deadbands
    public static final double kJoystickDeadband = 0.1;
    public static final double kTurnSensitivity = 6;

    // Joystick slew rate limiters to smooth erratic joystick motions, measured in units per second
    public static final double kJoystickSlewRateLimit = 3.0;

    // Fixed robot-relative nudge speed used by the driver POV bindings.
    public static final double kRobotRelativeNudgeSpeedMetersPerSec = Inches.of(11.0).in(Meters);

    // Demo teleop drive-to-pose target offset from the current pose.
    public static final double kAutopilotDemoXOffsetMeters = Feet.of(-10.0).in(Meters);

    // Override and Console Toggle Switches
    // Assumes this controller: https://www.amazon.com/gp/product/B00UUROWWK
    // Example from:
    // https://www.chiefdelphi.com/t/frc-6328-mechanical-advantage-2024-build-thread/442736/72
    public static final int DRIVER_SWITCH_0 = 1;
    public static final int DRIVER_SWITCH_1 = 2;
    public static final int DRIVER_SWITCH_2 = 3;

    public static final int OPERATOR_SWITCH_0 = 8;
    public static final int OPERATOR_SWITCH_1 = 9;
    public static final int OPERATOR_SWITCH_2 = 10;
    public static final int OPERATOR_SWITCH_3 = 11;
    public static final int OPERATOR_SWITCH_4 = 12;

    public static final int[] MULTI_TOGGLE = {4, 5};
  }

  /************************************************************************* */
  /** Driver Controller Button Constants *********************************** */
  public static final class ControllerButtonConstants {
    // Face buttons are named by physical position. RBSIController maps these to Xbox, PS4, or PS5
    // names for the detected controller.

    // DRIVER BUTTONS
    public static final Button ROBOT_RELATIVE = Button.EAST_FACE; // Xbox B / PS Circle
    public static final Button BRAKE = Button.SOUTH_FACE; // Xbox A / PS Cross
    public static final Button X_LOCK = Button.WEST_FACE; // Xbox X / PS Square
    public static final Button ZERO_GYRO = Button.NORTH_FACE; // Xbox Y / PS Triangle

    // Bumpers
    public static final Button RUN_FLYWHEEL = Button.RIGHT_BUMPER; // Xbox RB / PS R1
    public static final Button AUTOPILOT_DEMO = Button.LEFT_BUMPER; // Xbox LB / PS L1

    // Triggers
    public static final Axis LEFT_TRIGGER = Axis.LEFT_TRIGGER;
    public static final Axis RIGHT_TRIGGER = Axis.RIGHT_TRIGGER;

    // D-PAD POV Buttons
    public static final Button NUDGE_LEFT = Button.POV_LEFT;
    public static final Button NUDGE_RIGHT = Button.POV_RIGHT;
    public static final Button NUDGE_FORWARD = Button.POV_UP;
    public static final Button NUDGE_BACK = Button.POV_DOWN;

    // Pressing the joysticks
    public static final Button LEFT_STICK_BUTTON = Button.LEFT_STICK;
    public static final Button RIGHT_STICK_BUTTON = Button.RIGHT_STICK;

    public static final double kTriggerPressedThreshold = 0.5;
  }

  /************************************************************************* */
  /** Drive Base Constants ************************************************* */
  public static final class DrivebaseConstants {

    // Theoretical free speed (m/s) at 12v applied output;
    // IMPORTANT: Follow the AdvantageKit instructions for measuring the ACTUAL maximum linear speed
    // of YOUR ROBOT, and replace the estimate here with your measured value!
    public static final double kMaxLinearSpeedMetersPerSec = Feet.of(18).in(Meters);

    // Slip Current -- the current draw when the wheels start to slip
    // Measure this against a wall.  CHECK WITH THE CARPET AT AN ACTUAL EVENT!!!
    public static final double kSlipCurrentAmps = 40.0;

    // Characterized Wheel Radius (using the "Drive Wheel Radius Characterization" auto routine)
    public static final double kWheelRadiusMeters = Inches.of(2.000).in(Meters);

    // Maximum chassis accelerations desired for robot motion -- metric / radians.
    // Estimate from drivetrain characterization and robot physics, then tune on carpet.
    public static final double kMaxLinearAccelMetersPerSecSq = 4.0;

    // For Profiled PID Motion -- NEED TUNING!
    // Used in a variety of contexts, including PathPlanner and AutoPilot
    // Chassis (not module) across-the-field strafing motion
    public static final double kStrafeP = 5.0;
    public static final double kStrafeI = 0.0;
    public static final double kStrafeD = 0.0;
    // Chassis (not module) solid-body rotation
    public static final double kSpinP = 5.0;
    public static final double kSpinI = 0.0;
    public static final double kSpinD = 0.0;

    // Hold time on motor brakes when disabled
    public static final double kWheelLockTimeSecs = 10;

    // SysID characterization constants
    public static final double kSysIdMaxVoltage = 12.0;
    public static final double kSysIdDelaySecs = 3.0;
    public static final double kSysIdQuasistaticTimeoutSecs = 5.0;
    public static final double kSysIdDynamicTimeoutSecs = 3.0;
    public static final double kSysIdPreRunStopSecs = 1.0;
    public static final double kFeedforwardCharacterizationStartDelaySecs = 2.0;
    public static final double kFeedforwardCharacterizationRampRateVoltsPerSec = 0.1;
    public static final double kWheelRadiusCharacterizationStartDelaySecs = 1.0;
    public static final double kWheelRadiusCharacterizationMaxVelocityRadPerSec = 0.25;
    public static final double kWheelRadiusCharacterizationRampRateRadPerSecSq = 0.05;

    // Drive motor open-loop and closed-loop ramp periods for current smoothing
    //   Time from from 0 -> full duty
    public static final double kDriveClosedLoopRampPeriodSecs = 0.15;
    public static final double kDriveOpenLoopRampPeriodSecs = 0.25;

    public static final double kNominalVoltage = 12.0;

    // Default TalonFX Gains (Replaces what's in Phoenix X's Tuner Constants)
    // These are voltage-mode starting points for this custom IO layer, which configures TalonFX
    // drive feedback in mechanism rotations. The drive P/V values are CTRE Tuner's motor-side
    // defaults scaled by the drive reduction; characterize the robot before treating them as final.
    public static final double kDriveP = 0.70;
    public static final double kDriveD = 0.0;
    public static final double kDriveV = 0.83;
    public static final double kDriveA = 0.0;
    public static final double kDriveS = 0.20;
    public static final double kDriveT =
        SwerveConstants.kDriveGearRatio / DCMotor.getKrakenX60Foc(1).KtNMPerAmp;
    public static final double kSteerP = 100.0;
    public static final double kSteerD = 0.5;
    public static final double kSteerS = 0.1;

    // Odometry-related constants ==================================
    public static final double kPoseBufferHistorySecs = 1.5;
    // How aggressively to pull pose toward vision while DISABLED.
    // 0.10 = gentle, 0.25 = fairly quick, 1.0 = full snap.
    public static final double kDisabledVisionBlendAlpha = 0.15;
    // Optional: ignore obviously insane measurements while disabled.
    public static final double kDisabledVisionMaxJumpM = 2.0; // meters
    public static final double kDisabledVisionMaxJumpRad = Units.degreesToRadians(20.0);
    public static final double kDisabledVisionStale = 0.75; // seconds

    // Coast window config
    public static final double kDisabledCoastSeconds = 5.0;
    public static final double kDisabledCoastMinSeconds = 0.25;
    public static final double kDisabledVisionCoastBlendAlpha = 0.05;

    // "Stationary" detection config (tune)
    public static final double kStationaryMaxWheelDeltaM = 0.002; // 2mm per loop
    public static final double kStationaryMaxYawRateRadPerSec = 0.05; // ~3 deg/s
    public static final int kStationaryLoopsToEndCoast = 10; // ~0.20s @ 20ms
    public static final double kDisabledVisionIgnoreAfterDisableSec = 0.25; // 250ms
  }

  /************************************************************************* */
  /** Example Flywheel Mechanism Constants ********************************* */
  public static final class FlywheelConstants {

    // Mechanism idle mode
    public static final MotorIdleMode kIdleMode = MotorIdleMode.COAST; // BRAKE, COAST

    // Mechanism motor gear ratio
    public static final double kGearRatio = 1.5;
    public static final double kMaxVoltage = 12.0;

    // Flywheel motor open-loop and closed-loop ramp periods for current smoothing
    //   Time from from 0 -> full duty
    public static final double kClosedLoopRampPeriodSecs = 0.15;
    public static final double kOpenLoopRampPeriodSecs = 0.25;

    // SysId characterization settings
    public static final double kSysIdQuasistaticRampRateVoltsPerSec = 1.0;
    public static final double kSysIdDynamicStepVoltageVolts = 7.0;
    public static final double kSysIdTimeoutSecs = 10.0;

    // CTRE Motion Magic Velocity settings
    public static final double kMotionMagicAccelerationRotPerSecSq = 400.0;
    public static final double kMotionMagicJerkRotPerSecCubed = 4000.0;

    // MODE == REAL / REPLAY
    // Feedforward constants
    public static final double kRealS = 0.1;
    public static final double kRealV = 0.05;
    public static final double kRealA = 0.0;
    // Feedback (PID) constants
    public static final double kRealP = 1.0;
    public static final double kRealD = 0.0;

    // MODE == SIM
    // Feedforward constants
    public static final double kSimS = 0.0;
    public static final double kSimV = 0.03;
    public static final double kSimA = 0.0;
    // Feedback (PID) constants
    public static final double kSimP = 0.0;
    public static final double kSimD = 0.0;
    // Simulation plant constants
    public static final double kSimGearing = 1.0;
    public static final double kSimMomentOfInertiaKgMetersSq =
        0.5 * Pounds.of(1.5).in(Kilograms) * Math.pow(Inches.of(4.0).in(Meters), 2.0);
  }

  /************************************************************************* */
  /** Place Other Mechanism Constant Classes Here ************************** */
  // public static class Mechanism1Constants {}
  // public static class Mechanism2Constants {}
  // ...

  /************************************************************************* */
  /** (Semi-)Autonomous Action Constants *********************************** */
  public static final class AutoConstants {

    // ********** PATHPLANNER CONSTANTS *******************
    // PathPlanner Config constants
    public static final RobotConfig kPathPlannerConfig =
        new RobotConfig(
            RobotConstants.kMass.in(Kilograms),
            RobotConstants.kMomentOfInertiaKgMetersSq,
            new ModuleConfig(
                DrivebaseConstants.kWheelRadiusMeters,
                DrivebaseConstants.kMaxLinearSpeedMetersPerSec,
                RobotConstants.kWheelCoefficientOfFriction,
                DCMotor.getKrakenX60Foc(1).withReduction(SwerveConstants.kDriveGearRatio),
                DrivebaseConstants.kSlipCurrentAmps,
                1),
            Drive.getModuleTranslations());

    // Alternatively, we can build this from the PathPlanner GUI:
    // public static final RobotConfig kPathPlannerConfig = RobotConfig.fromGUISettings();

    // ********** CHOREO CONSTANTS ************************
    // Drive and Turn PID constants used for ChoreO
    public static final PIDConstants kChoreoDrivePID = new PIDConstants(10.0, 0.0, 0.0);
    public static final PIDConstants kChoreoSteerPID = new PIDConstants(7.5, 0.0, 0.0);

    // ********** AUTOPILOT CONSTANTS *********************
    // Autopilot (Drive to Pose in Teleop) Constraints
    // see https://therekrab.github.io/autopilot/usage.html

    // Acceleration and Jerk to be applied
    private static final APConstraints kAPConstraints =
        new APConstraints().withAcceleration(5.0).withJerk(2.0);

    // Motion profile for drive to pose
    private static final APProfile kAPProfile =
        new APProfile(kAPConstraints)
            .withErrorXY(Centimeters.of(2))
            .withErrorTheta(Degrees.of(0.5))
            .withBeelineRadius(Centimeters.of(8));

    // Autopilot object to be used for specific commands
    public static final Autopilot kAutopilot = new Autopilot(kAPProfile);
  }

  /************************************************************************* */
  /** Vision Constants (Assuming PhotonVision) ***************************** */
  public static class VisionConstants {

    public static final Set<Integer> kTrustedTags =
        Set.of(2, 3, 4, 5, 8, 9, 10, 11, 18, 19, 20, 21, 24, 25, 26, 27); // HUB AprilTags

    // Noise scaling factors (lower = more trusted)
    public static final double kTrustedTagStdDevScale = 0.6; // 40% more weight
    public static final double kUntrustedTagStdDevScale = 1.3; // 30% less weight

    // Optional: if true, reject observations that contain no trusted tags
    public static final boolean kRequireTrustedTag = false;

    // AprilTag Identification Constants
    public static final double kAmbiguityThreshold = 0.4;
    public static final double kTargetLogTimeSecs = 0.1;
    public static final double kFieldBorderMargin = 0.5;
    public static final double kZMargin = 0.75;
    public static final double kXYZStdDevCoefficient = 0.005;
    public static final double kThetaStdDevCoefficient = 0.01;

    // Basic filtering thresholds
    public static final double kMaxAmbiguity = 0.3;
    public static final double kMaxZErrorMeters = 0.75;

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static final double kLinearStdDevBaseline = 0.02; // Meters
    public static final double kAngularStdDevBaseline = 0.06; // Radians

    // Multipliers to apply for MegaTag 2 observations
    public static final double kLinearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
    public static final double kAngularStdDevMegatag2Factor =
        Double.POSITIVE_INFINITY; // No rotation data available
  }

  /************************************************************************* */
  /** Vision Camera Poses ************************************************** */
  public static final class Cameras {
    public record CameraConfig(
        String name,
        Transform3d robotToCamera,
        double stdDevFactor,
        SimCameraProperties simProps) {}

    // Camera Configuration Records
    // (ONLY USED FOR PHOTONVISION -- Limelight: configure in web UI instead)
    // Example Cameras are mounted in the back corners, 18" up from the floor, facing sideways
    public static final CameraConfig[] ALL = {
      new CameraConfig(
          "Photon_BW7",
          new Transform3d(
              Inches.of(-13.0),
              Inches.of(13.0),
              Inches.of(12.0),
              new Rotation3d(0.0, 0.0, Math.PI / 2)),
          1.0,
          new SimCameraProperties() {
            {
              setCalibration(1280, 800, Rotation2d.fromDegrees(120));
              setCalibError(0.25, 0.08);
              setFPS(30);
              setAvgLatencyMs(20);
              setLatencyStdDevMs(5);
            }
          }),
      //
      // new CameraConfig(
      //     "camera_1",
      //     new Transform3d(
      //         Inches.of(-13.0),
      //         Inches.of(-13.0),
      //         Inches.of(12.0),
      //         new Rotation3d(0.0, 0.0, -Math.PI / 2)),
      //     1.0,
      //     new SimCameraProperties() {
      //       {
      //         setCalibration(1280, 800, Rotation2d.fromDegrees(120));
      //         setCalibError(0.25, 0.08);
      //         setFPS(30);
      //         setAvgLatencyMs(20);
      //         setLatencyStdDevMs(5);
      //       }
      //     }),

      // ... And more, if needed
    };
  }

  /************************************************************************* */
  /** Deploy Directory Location Constants ********************************** */
  public static final class DeployConstants {
    public static final String kAprilTagDir = "apriltags";
    public static final String kChoreoDir = "choreo";
    public static final String kPathPlannerDir = "pathplanner";
    public static final String kYagslDir = "swerve";
  }

  /***************************************************************************/
  /** Getter functions -- do not modify ************************************ */
  /** Get the current robot */
  public static RobotType getRobot() {
    if (!disableHAL && RobotBase.isReal() && robotType == RobotType.SIMBOT) {
      new Alert(
              "Invalid robot selected, using competition robot as default.", Alert.AlertType.ERROR)
          .set(true);
      robotType = RobotType.COMPBOT;
    }
    return robotType;
  }

  /** Get the current robot mode */
  public static Mode getMode() {
    return switch (robotType) {
      case DEVBOT1, DEVBOT2, COMPBOT -> RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;
      case SIMBOT -> Mode.SIM;
    };
  }

  /** Return whether this is pure simulation */
  public static boolean isPureSim() {
    boolean isReplay = Logger.hasReplaySource();
    return getMode() == Mode.SIM && !isReplay;
  }

  /** Return whether live tuning/debug logging is enabled. */
  public static boolean isTuningMode() {
    return kTuningMode;
  }

  /** Get the current swerve drive type */
  public static SwerveType getSwerveType() {
    return swerveType;
  }

  /** Get the current autonomous path planning type */
  public static AutoType getAutoType() {
    return autoType;
  }

  /** Get the current autonomous path planning type */
  public static VisionType getVisionType() {
    return visionType;
  }

  /** Get the current CTRE/Phoenix Pro License state */
  public static CTREPro getPhoenixPro() {
    return phoenixPro;
  }

  /** Get the current AprilTag layout type. */
  public static AprilTagLayoutType getAprilTagLayoutType() {
    return FieldConstants.defaultAprilTagType;
  }
}
