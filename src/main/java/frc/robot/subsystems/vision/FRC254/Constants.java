// Copyright (c) 2024 FRC 254
// https://github.com/team254
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

package frc.robot.subsystems.vision.FRC254;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.util.RobotDeviceId;
import frc.robot.util.ServoMotorSubsystemConfig;
import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.Enumeration;

/** Class to store all constants for robot code. */
public class Constants {
  public static final String kCanBusCanivore = "canivore";
  public static boolean kIsReplay = false;
  public static final String kPracticeBotMacAddress = "00:80:2F:33:D1:4B";
  public static boolean kIsPracticeBot = hasMacAddress(kPracticeBotMacAddress);

  public static final double kSteerJoystickDeadband = 0.05;

  public static final ClosedLoopRampsConfigs makeDefaultClosedLoopRampConfig() {
    return new ClosedLoopRampsConfigs()
        .withDutyCycleClosedLoopRampPeriod(0.02)
        .withTorqueClosedLoopRampPeriod(0.02)
        .withVoltageClosedLoopRampPeriod(0.02);
  }

  public static final OpenLoopRampsConfigs makeDefaultOpenLoopRampConfig() {
    return new OpenLoopRampsConfigs()
        .withDutyCycleOpenLoopRampPeriod(0.02)
        .withTorqueOpenLoopRampPeriod(0.02)
        .withVoltageOpenLoopRampPeriod(0.02);
  }

  public static final class DriveConstants {
    public static final double kDriveMaxSpeed = 4.43676260556;
    public static final double kDriveMaxAngularRate = Units.degreesToRadians(360 * 1.15);
    public static final double kHeadingControllerP = 3.5;
    public static final double kHeadingControllerI = 0;
    public static final double kHeadingControllerD = 0;
  }

  public static final class SensorConstants {
    public static final int kAmpPostChopstickSensorPort = 4;
    public static final int kShooterStage1BannerSensorPort = 3;
    public static final int kAmpBannerSensorPort = 2;
    public static final int kFeederBannerSensorPort = 1;
    public static final int kIntakeBannerSensorPort = 0;
    public static final double kShooterDebounceTime = 0.01;
    public static final double kAmpDebounceTime = 0.01;
    public static final double kFeederDebounceTime = 0.01;
    public static final double kIntakeDebounceTime = 0.01;
  }

  public static final class ClimberConstants {
    public static final RobotDeviceId kClimberTalonCanID =
        new RobotDeviceId(24, kCanBusCanivore, 0);
    public static final double kClimberP = kIsPracticeBot ? 1.0 : 1.0;
    public static final double kForwardMaxPositionRotations = kIsPracticeBot ? 119.0 : 132.0;
    public static final double kHooksUpPositionRotations = kForwardMaxPositionRotations * 0.9;
    public static final double kStageHooksRotations = kForwardMaxPositionRotations * 0.4;
    public static final double kClimbClimbedPositionToleranceRotations =
        kForwardMaxPositionRotations * 0.1;
    public static final double kPositionToleranceRotations = 2.0;
    public static final double kClimberGearRatio = 1.0 / (10.0);
    public static double kReverseMinPositionRotations = 0.0;
  }

  public static final class ElevatorConstants {
    public static final double ElevatorMinPositionRotations = 0.0;
    public static final double ElevatorMaxPositionRotations = 15.356933;
    public static final double ElevatorMaxHeightInches = 16.5;
    public static final double kElevatorGearRatio = (11.0 / 36.0) * (18. / 15.);
    public static final double kElevatorPositionToleranceRotations = 0.1;
    public static final double kAmpScoringHeightInches = 16.0;
    public static final double kElevatorHomeHeightInches = 0.0;
    public static final double kIntakeFromSourceHeightInches = 14.5;
    public static final double kElevatorPositioningToleranceInches = 0.5;
    public static final double kClimbHeightInches = 16.0;
    public static final double kSpoolDiameter = Units.inchesToMeters(0.940);
  }

  // Intake Constants
  public static final class IntakeConstants {
    public static final double kIntakeDutyCycleIntake = 1.;
    public static final double kIntakeDutyCycleIntakeFromSource = 1.0;
    public static final double kIntakeDutyCycleExhuast = -1.0;
  }

  // Feeder Constants
  public static final class FeederConstants {
    public static final double kFeederRPSIntake = 70.0;
    public static final boolean kRunClockwise = true;
  }

  public static class AmpConstants {
    public static final double kAmpIntakeFromSourceDutyCycle = -1.0;
    public static final double kAmpExhaustToStageDutyCycle = 0.85;
    public static final double kAmpSlowlyStageDutyCycle = 0.1;

    public static final double kAmpScoreDutyCycle = -1.0;

    // Number of rotations to turn after beam break is tripped (rising edge) to stow
    // the note prior to scoring
    public static final double kAmpChopsticksStageRotations = 6.0;
    public static final double kTrapChopsticksStageRotations = 10.0;
    public static final double kAmpChopsticksGoBackRotations = -1.75;
    public static final double kAmpChopsticksGoBackRotationsTolerance = 0.25;
    public static final double kAmpChopsticksStageRotationsTolerance = 0.1;
  }

  public static final ServoMotorSubsystemConfig kAmpConfig = new ServoMotorSubsystemConfig();

  static {
    kAmpConfig.name = "Amp";
    kAmpConfig.talonCANID = new RobotDeviceId(25, kCanBusCanivore, 0);
    kAmpConfig.fxConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    kAmpConfig.fxConfig.Audio.BeepOnBoot = false;
    kAmpConfig.fxConfig.Audio.BeepOnConfig = false;

    kAmpConfig.fxConfig.Slot0.kS = 0.015 * 12.0;
    kAmpConfig.fxConfig.Slot0.kP = 0.3 * 12.0;
    kAmpConfig.fxConfig.Slot0.kV = 0.00925 * 12.0;
    kAmpConfig.fxConfig.Slot0.kA = 0.0001 * 12.0;
    kAmpConfig.fxConfig.MotionMagic.MotionMagicAcceleration = 500.0;
    kAmpConfig.fxConfig.MotionMagic.MotionMagicCruiseVelocity = 50.0;

    kAmpConfig.fxConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    kAmpConfig.fxConfig.CurrentLimits.SupplyCurrentLimit = 60.0;

    kAmpConfig.fxConfig.CurrentLimits.StatorCurrentLimit = 80.0;
    kAmpConfig.fxConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    kAmpConfig.fxConfig.ClosedLoopRamps = makeDefaultClosedLoopRampConfig();
    kAmpConfig.fxConfig.OpenLoopRamps = makeDefaultOpenLoopRampConfig();
  }

  public static final class ShooterConstants {
    public static final Rotation2d kTurretToShotCorrection =
        new Rotation2d(Units.degreesToRadians(1.5));

    public static final double kPoopMaxApexHeight = Units.inchesToMeters(160.0);

    public static final double kStage2ShooterWheelDiameter = Units.inchesToMeters(3.0); // in
    public static final double kStage1ShooterWheelDiameter = Units.inchesToMeters(2.0); // in

    public static final double kRingLaunchVelMetersPerSecPerRotPerSec = 0.141;
    public static final double kRingLaunchLiftCoeff = 0.013; // Multiply by v^2 to get lift accel
    public static final double kShooterStage2RPSShortRange = 120.0; // rot/s
    public static final double kShooterStage2MaxShortRangeDistance = 2.0;
    public static final double kShooterStage2MinLongRangeDistance = 3.0;
    public static final double kShooterStage2RPSLongRange = 120.0; // rot/s
    public static final double kShooterStage2RPSCap = 130.0; // rot/s
    public static final double kShooterStage1RPS = 70.0; // rot/s
    public static final double kShooterStage2Epsilon = 3.0;
    public static final double kShooterSpinupStage1RPS = 0.0;

    public static final double kShooterStage1IntakeRPS = 4.0;
    public static final double kShooterStage1ExhaustRPS = -10.0;

    public static final double kFenderShotRPS = 100.0;
    public static final double kPreloadShotRPS = 90.0;

    public static final double kBottomRollerSpeedupFactor =
        1.0; // multiplied to all setpoints to determine how
    // much
    // extra power to give the bottom roller. >1.0 =
    // faster
    // bottom roller
    public static final double kTopRollerSpeedupFactor = 1.0;
  }

  public static final double kJoystickThreshold = 0.1;
  public static final int kDriveGamepadPort = 0;

  // Controls
  public static final boolean kForceDriveGamepad = true;
  public static final int kGamepadAdditionalControllerPort = 1;
  public static final int kOperatorControllerPort = 2;
  public static final int kMainThrottleJoystickPort = 0;
  public static final int kMainTurnJoystickPort = 1;
  public static final double kDriveJoystickThreshold = 0.03;

  // Limelight constants

  // TURRET LIMELIGHT
  // Pitch angle: How many radians the camera is pitched up around Y axis. 0 is
  // looking straight ahead, +is nodding up.
  public static final double kCameraPitchDegrees = kIsPracticeBot ? 28.0 : 27.5;
  public static final double kCameraPitchRads = Units.degreesToRadians(kCameraPitchDegrees);
  public static final double kCameraHeightOffGroundMeters =
      kIsPracticeBot ? Units.inchesToMeters(11.181) : Units.inchesToMeters(11.181);
  public static final double kImageCaptureLatency = 11.0; // milliseconds
  public static final double kLimelightTransmissionTimeLatency = 0.0; // milliseconds
  public static final String kLimelightTableName = "limelight-turret";
  // Distance from turret center to camera lens in X axis (straight into lens)
  public static final double kTurretToCameraX =
      kIsPracticeBot ? Units.inchesToMeters(5.834) : Units.inchesToMeters(5.834);
  // Distance from turret center to camera lens in Y
  public static final double kTurretToCameraY = 0;

  // ELEVATOR LIMELIGHT
  public static final String kLimelightBTableName = "limelight-eleva";
  public static final double kCameraBPitchDegrees = kIsPracticeBot ? 15.0 : 16.0;
  public static final double kCameraBPitchRads = Units.degreesToRadians(kCameraBPitchDegrees);
  public static final double kCameraBRollDegrees = kIsPracticeBot ? 0.0 : 0.0;
  public static final double kCameraBRollRads = Units.degreesToRadians(kCameraBRollDegrees);
  public static final double kCameraBHeightOffGroundMeters =
      kIsPracticeBot
          ? Units.inchesToMeters(19.477)
          : Units.inchesToMeters(19.477); // verify for practice
  // Distance from turret center to camera lens in X axis (straight into lens)
  public static final double kTurretToCameraBX =
      kIsPracticeBot
          ? Units.inchesToMeters(14.882)
          : Units.inchesToMeters(14.882); // verify for practice
  // Distance from turret center to camera lens in Y
  public static final double kTurretToCameraBY = 0;

  public static final double kTurretToRobotCenterX = Units.inchesToMeters(2.3115);
  public static final double kTurretToRobotCenterY = 0;
  public static final Transform2d kTurretToRobotCenter =
      new Transform2d(
          new Translation2d(Constants.kTurretToRobotCenterX, Constants.kTurretToRobotCenterY),
          new Rotation2d());
  public static final Rotation2d kCameraYawOffset = new Rotation2d(0);

  // April Tag Layout
  public static final AprilTagFieldLayout kAprilTagLayout =
      AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

  public static final double kFieldWidthMeters =
      kAprilTagLayout.getFieldWidth(); // distance between field walls,
  // 8.211m
  public static final double kFieldLengthMeters =
      kAprilTagLayout.getFieldLength(); // distance between driver station
  // walls, 16.541m

  public static final Pose2d kBlueAmpPose = new Pose2d(1.820, 7.680, Rotation2d.fromDegrees(90.0));
  public static final Pose2d kRedAmpPose =
      new Pose2d(
          kFieldWidthMeters - kBlueAmpPose.getX(),
          kBlueAmpPose.getY(),
          Rotation2d.fromDegrees(180 - kBlueAmpPose.getRotation().getDegrees())); // X 14.7345

  public static final Pose2d kBlueClimbPoseFeed =
      new Pose2d(4.4, 3.3, Rotation2d.fromDegrees(60.0));
  public static final Pose2d kBlueClimbPoseAmp =
      new Pose2d(4.43, 4.95, Rotation2d.fromDegrees(-60.0));
  public static final Pose2d kBlueClimbPoseMidline =
      new Pose2d(5.8, 4.1, Rotation2d.fromDegrees(180.0));
  public static final Pose2d kRedClimbPoseFeed =
      new Pose2d(
          kFieldWidthMeters - kBlueClimbPoseFeed.getX(),
          kBlueClimbPoseFeed.getY(),
          Rotation2d.fromDegrees(180 - kBlueClimbPoseFeed.getRotation().getDegrees()));
  public static final Pose2d kRedClimbPoseAmp =
      new Pose2d(
          kFieldWidthMeters - kBlueClimbPoseAmp.getX(),
          kBlueClimbPoseAmp.getY(),
          Rotation2d.fromDegrees(180 - kBlueClimbPoseAmp.getRotation().getDegrees()));
  public static final Pose2d kRedClimbPoseMidline =
      new Pose2d(
          kFieldWidthMeters - kBlueClimbPoseMidline.getX(),
          kBlueClimbPoseMidline.getY(),
          Rotation2d.fromDegrees(180 - kBlueClimbPoseMidline.getRotation().getDegrees()));
  public static final Translation3d kRedSpeakerPose =
      new Translation3d(
          Constants.kAprilTagLayout.getTagPose(4).get().getX(),
          Constants.kAprilTagLayout.getTagPose(4).get().getY(),
          2.045);
  public static final Translation3d kBlueSpeakerPose =
      new Translation3d(
          Constants.kAprilTagLayout.getTagPose(7).get().getX(),
          Constants.kAprilTagLayout.getTagPose(7).get().getY(),
          2.045);
  public static final Translation3d kRedSpeakerTopPose =
      new Translation3d(
          Constants.kAprilTagLayout.getTagPose(4).get().getX() - Units.inchesToMeters(20.057),
          Constants.kAprilTagLayout.getTagPose(4).get().getY(),
          Constants.kAprilTagLayout.getTagPose(4).get().getZ() + Units.inchesToMeters(32.563));
  public static final Translation3d kBlueSpeakerTopPose =
      new Translation3d(
          Constants.kAprilTagLayout.getTagPose(7).get().getX() + Units.inchesToMeters(20.057),
          Constants.kAprilTagLayout.getTagPose(7).get().getY(),
          Constants.kAprilTagLayout.getTagPose(7).get().getZ() + Units.inchesToMeters(32.563));

  public static final Translation2d kBlueStageCenterPose =
      new Translation2d(4.83, kFieldWidthMeters / 2.0);
  public static final Translation2d kRedStageCenterPose =
      new Translation2d(
          kFieldLengthMeters - kBlueStageCenterPose.getX(), kBlueStageCenterPose.getY());
  public static final double kBlueSpeakerToStageAutoSwitchX = 7.0;
  public static final double kRedSpeakerToStageAutoSwitchX =
      kFieldLengthMeters - kBlueSpeakerToStageAutoSwitchX;

  public static final double kNoteReleaseHeight = Units.inchesToMeters(22.183);
  public static final Pose3d kLeftRedSpeakerPose =
      new Pose3d(
          Constants.kAprilTagLayout.getTagPose(4).get().getX(), 5.875, 2.04, new Rotation3d());
  public static final Pose3d kRightRedSpeakerPose =
      new Pose3d(
          Constants.kAprilTagLayout.getTagPose(4).get().getX(), 5.29, 2.04, new Rotation3d());
  public static final Pose3d kLeftBlueSpeakerPose =
      new Pose3d(
          Constants.kAprilTagLayout.getTagPose(7).get().getX(), 5.875, 2.04, new Rotation3d());
  public static final Pose3d kRightBlueSpeakerPose =
      new Pose3d(
          Constants.kAprilTagLayout.getTagPose(7).get().getX(), 5.29, 2.04, new Rotation3d());
  public static final double kSpeakerLengthMeters =
      kLeftRedSpeakerPose.getY() - kRightRedSpeakerPose.getY();

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 4.75;
    public static final double kMaxAccelerationMetersPerSecondSquared = 4.85;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 6.0;
    public static final double kPYController = 2.0;
    public static final double kPThetaController = 4.0;

    public static final double kTranslationKa = 0.0;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

    public static final PathConstraints kAutoAlignPathConstraints =
        new PathConstraints(
            kMaxSpeedMetersPerSecond,
            kMaxAccelerationMetersPerSecondSquared,
            kMaxAngularSpeedRadiansPerSecond,
            kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class LEDConstants {
    public static final RobotDeviceId kCANdleId = new RobotDeviceId(30, kCanBusCanivore, 0);
    public static final int kNonCandleLEDCount = 30;
    public static final int kCandleLEDCount = 8;
    public static final int kMaxLEDCount = kNonCandleLEDCount + kCandleLEDCount;
  }

  /**
   * Check if this system has a certain mac address in any network device.
   *
   * @param mac_address Mac address to check.
   * @return true if some device with this mac address exists on this system.
   */
  public static boolean hasMacAddress(final String mac_address) {
    try {
      Enumeration<NetworkInterface> nwInterface = NetworkInterface.getNetworkInterfaces();
      while (nwInterface.hasMoreElements()) {
        NetworkInterface nis = nwInterface.nextElement();
        if (nis == null) {
          continue;
        }
        StringBuilder device_mac_sb = new StringBuilder();
        System.out.println("hasMacAddress: NIS: " + nis.getDisplayName());
        byte[] mac = nis.getHardwareAddress();
        if (mac != null) {
          for (int i = 0; i < mac.length; i++) {
            device_mac_sb.append(String.format("%02X%s", mac[i], (i < mac.length - 1) ? ":" : ""));
          }
          String device_mac = device_mac_sb.toString();
          System.out.println(
              "hasMacAddress: NIS " + nis.getDisplayName() + " device_mac: " + device_mac);
          if (mac_address.equals(device_mac)) {
            System.out.println("hasMacAddress: ** Mac address match! " + device_mac);
            return true;
          }
        } else {
          System.out.println("hasMacAddress: Address doesn't exist or is not accessible");
        }
      }

    } catch (SocketException e) {
      e.printStackTrace();
    }
    return false;
  }
}
