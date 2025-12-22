// Copyright (c) 2024-2025 FRC 254
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
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.Arrays;
import java.util.Enumeration;

public class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;
  public static final SimControllerType kSimControllerType = SimControllerType.XBOX;

  public enum SimControllerType {
    XBOX,
    DUAL_SENSE
  }

  public enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final String kCanBusDrivebaseClimberCanivore = "drivebase-climber";
  public static final String kCanBusSuperstructureCanivore = "superstructure";
  public static boolean kIsReplay = false;
  public static final String kPracticeBotMacAddress = "00:80:2F:33:BF:BB";
  public static boolean kIsPracticeBot = hasMacAddress(kPracticeBotMacAddress);

  public static final double kSteerJoystickDeadband = 0.05;
  public static final double kRobotWidth = Units.inchesToMeters(35.625);
  public static final double kRobotDiagonal = Math.sqrt(2.0) * kRobotWidth;
  public static final double kRobotMassKg = Units.lbsToKilograms(147.92);
  public static final double kRobotMomentOfInertia = 2 * 9.38; // kg * m^2
  public static final double kCOGHeightMeters = Units.inchesToMeters(0.0);

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

  // April Tag Layout
  public static final AprilTagFieldLayout kAprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
  public static final int[] kAllowedTagIDs = {17, 18, 19, 20, 21, 22, 6, 7, 8, 9, 10, 11};
  public static final AprilTagFieldLayout kAprilTagLayoutReefsOnly =
      new AprilTagFieldLayout(
          kAprilTagLayout.getTags().stream()
              .filter(tag -> Arrays.stream(kAllowedTagIDs).anyMatch(element -> element == tag.ID))
              .toList(),
          kAprilTagLayout.getFieldLength(),
          kAprilTagLayout.getFieldWidth());

  public static final double kFieldWidthMeters = kAprilTagLayout.getFieldWidth();
  public static final double kFieldLengthMeters = kAprilTagLayout.getFieldLength();

  public static final double kReefRadius = 0.9604; // m

  public static final double kMidlineBuffer = 1.0;

  // Limelight constants
  public static final class VisionConstants {

    // Large variance used to downweight unreliable vision measurements
    public static final double kLargeVariance = 1e6;

    // Standard deviation constants
    public static final int kMegatag1XStdDevIndex = 0;
    public static final int kMegatag1YStdDevIndex = 1;
    public static final int kMegatag1YawStdDevIndex = 5;

    // Standard deviation array indices for Megatag2
    public static final int kMegatag2XStdDevIndex = 6;
    public static final int kMegatag2YStdDevIndex = 7;
    public static final int kMegatag2YawStdDevIndex = 11;

    // Validation constants
    public static final int kExpectedStdDevArrayLength = 12;

    public static final int kMinFiducialCount = 1;

    // Camera A (Left-side Camera)
    public static final double kCameraAPitchDegrees = 20.0;
    public static final double kCameraAPitchRads = Units.degreesToRadians(kCameraAPitchDegrees);
    public static final double kCameraAHeightOffGroundMeters = Units.inchesToMeters(8.3787);
    public static final String kLimelightATableName = "limelight-left";
    public static final double kRobotToCameraAForward = Units.inchesToMeters(7.8757);
    public static final double kRobotToCameraASide = Units.inchesToMeters(-11.9269);
    public static final Rotation2d kCameraAYawOffset = Rotation2d.fromDegrees(0.0);
    public static final Transform2d kRobotToCameraA =
        new Transform2d(
            new Translation2d(kRobotToCameraAForward, kRobotToCameraASide), kCameraAYawOffset);

    // Camera B (Right-side camera)
    public static final double kCameraBPitchDegrees = 20.0;
    public static final double kCameraBPitchRads = Units.degreesToRadians(kCameraBPitchDegrees);
    public static final double kCameraBHeightOffGroundMeters = Units.inchesToMeters(8.3787);
    public static final String kLimelightBTableName = "limelight-right";
    public static final double kRobotToCameraBForward = Units.inchesToMeters(7.8757);
    public static final double kRobotToCameraBSide = Units.inchesToMeters(11.9269);
    public static final Rotation2d kCameraBYawOffset = Rotation2d.fromDegrees(0.0);
    public static final Transform2d kRobotToCameraB =
        new Transform2d(
            new Translation2d(kRobotToCameraBForward, kRobotToCameraBSide), kCameraBYawOffset);

    // Vision processing constants
    public static final double kDefaultAmbiguityThreshold = 0.19;
    public static final double kDefaultYawDiffThreshold = 5.0;
    public static final double kTagAreaThresholdForYawCheck = 2.0;
    public static final double kTagMinAreaForSingleTagMegatag = 1.0;
    public static final double kDefaultZThreshold = 0.2;
    public static final double kDefaultNormThreshold = 1.0;
    public static final double kMinAmbiguityToFlip = 0.08;

    public static final double kCameraHorizontalFOVDegrees = 81.0;
    public static final double kCameraVerticalFOVDegrees = 55.0;
    public static final int kCameraImageWidth = 1280;
    public static final int kCameraImageHeight = 800;

    public static final double kScoringConfidenceThreshold = 0.7;

    // NetworkTables constants
    public static final String kBoundingBoxTableName = "BoundingBoxes";
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
