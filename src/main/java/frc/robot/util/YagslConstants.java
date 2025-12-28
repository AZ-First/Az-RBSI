// Copyright (c) 2024-2026 Az-FIRST
// http://github.com/AZ-First
//
// Use of this source code is governed by a BSD
// license that can be found in the AdvantageKit-License.md file
// at the root directory of this project.
//
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Constants.DeployConstants;
import java.io.File;
import java.util.Arrays;
import java.util.List;
import swervelib.parser.json.ModuleJson;
import swervelib.parser.json.PIDFPropertiesJson;
import swervelib.parser.json.PhysicalPropertiesJson;
import swervelib.parser.json.SwerveDriveJson;

/**
 * The YAGSL swerve drive constants class
 *
 * <p>The purpose of this class is to convert the YAGSL-based JSON configuration file data into the
 * same constants format as produced by the Phoenix Tuner X's ``TunerConstants.java`` file. The goal
 * here is to make implenentation in the drive class easier.
 */
public class YagslConstants {

  // Define the internal YAGSL objects we will read into
  private static final File yagslDir =
      new File(Filesystem.getDeployDirectory(), DeployConstants.yagslDir);
  public static final SwerveDriveJson swerveDriveJson; // Needed by the Accelerometer subsystem
  private static final PIDFPropertiesJson pidfPropertiesJson;
  private static final PhysicalPropertiesJson physicalPropertiesJson;
  private static final ModuleJson[] moduleJsons;

  // Use a static <JAVA THING> to read in the YAGSL JSON files with error catching
  static {
    SwerveDriveJson tempSwerveJson = null;
    PIDFPropertiesJson tempPdifJson = null;
    PhysicalPropertiesJson tempPhysicalJson = null;
    ModuleJson[] tempModuleJsons = null;

    try {
      tempSwerveJson =
          new ObjectMapper()
              .readValue(new File(yagslDir, "swervedrive.json"), SwerveDriveJson.class);
      tempPdifJson =
          new ObjectMapper()
              .readValue(
                  new File(yagslDir, "modules/pidfproperties.json"), PIDFPropertiesJson.class);
      tempPhysicalJson =
          new ObjectMapper()
              .readValue(
                  new File(yagslDir, "modules/physicalproperties.json"),
                  PhysicalPropertiesJson.class);

    } catch (Exception e) {
      throw new RuntimeException(e);
    }
    swerveDriveJson = tempSwerveJson;
    pidfPropertiesJson = tempPdifJson;
    physicalPropertiesJson = tempPhysicalJson;

    tempModuleJsons = new ModuleJson[swerveDriveJson.modules.length];
    try {
      for (int i = 0; i < tempModuleJsons.length; i++) {
        // moduleConfigs.put(swerveDriveJson.modules[i], i);
        File moduleFile = new File(yagslDir, "modules/" + swerveDriveJson.modules[i]);
        assert moduleFile.exists();
        tempModuleJsons[i] = new ObjectMapper().readValue(moduleFile, ModuleJson.class);
      }
    } catch (Exception e) {
      throw new RuntimeException(e);
    }
    moduleJsons = tempModuleJsons;
  }

  // Define the Tuner X - like constants from the YAGSL objects

  // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
  // This may need to be tuned to your individual robot
  public static final double kCoupleRatio = 3.5;

  public static final double kDriveGearRatio =
      physicalPropertiesJson.conversionFactors.drive.gearRatio;
  public static final double kSteerGearRatio =
      physicalPropertiesJson.conversionFactors.angle.gearRatio;
  public static final double kWheelRadiusInches =
      physicalPropertiesJson.conversionFactors.drive.diameter / 2.0;

  public static final String kCANbusName = swerveDriveJson.imu.canbus;
  public static final int kPigeonId = swerveDriveJson.imu.id;

  // These are only used for simulation
  public static final double kSteerInertia = 0.004;
  public static final double kDriveInertia = 0.025;
  // Simulated voltage necessary to overcome friction
  public static final double kSteerFrictionVoltage = 0.25;
  public static final double kDriveFrictionVoltage = 0.25;

  // Current / Voltage Limits
  public static final double kSteerCurrentLimit = physicalPropertiesJson.currentLimit.angle;
  public static final double kDriveCurrentLimit = physicalPropertiesJson.currentLimit.drive;
  public static final double kOptimalVoltage = physicalPropertiesJson.optimalVoltage;

  // The modules themselves
  static List<String> moduleList = Arrays.asList(swerveDriveJson.modules);

  // Front Left
  static ModuleJson flModule = moduleJsons[moduleList.indexOf("frontleft.json")];
  public static final int kFrontLeftDriveMotorId = flModule.drive.id;
  public static final int kFrontLeftSteerMotorId = flModule.angle.id;
  public static final int kFrontLeftEncoderId = flModule.encoder.id;
  public static final String kFrontLeftDriveCanbus = flModule.drive.canbus;
  public static final String kFrontLeftSteerCanbus = flModule.angle.canbus;
  public static final String kFrontLeftEncoderCanbus = flModule.encoder.canbus;
  public static final String kFrontLeftDriveType = flModule.drive.type;
  public static final String kFrontLeftSteerType = flModule.angle.type;
  public static final String kFrontLeftEncoderType = flModule.encoder.type;
  public static final double kFrontLeftEncoderOffset = flModule.absoluteEncoderOffset;
  public static final boolean kFrontLeftDriveInvert = flModule.inverted.drive;
  public static final boolean kFrontLeftSteerInvert = flModule.inverted.angle;
  public static final boolean kFrontLeftEncoderInvert = flModule.absoluteEncoderInverted;
  public static final double kFrontLeftXPosInches = flModule.location.left;
  public static final double kFrontLeftYPosInches = flModule.location.front;

  // Front Right
  static ModuleJson frModule = moduleJsons[moduleList.indexOf("frontright.json")];
  public static final int kFrontRightDriveMotorId = frModule.drive.id;
  public static final int kFrontRightSteerMotorId = frModule.angle.id;
  public static final int kFrontRightEncoderId = frModule.encoder.id;
  public static final String kFrontRightDriveCanbus = frModule.drive.canbus;
  public static final String kFrontRightSteerCanbus = frModule.angle.canbus;
  public static final String kFrontRightEncoderCanbus = frModule.encoder.canbus;
  public static final String kFrontRightDriveType = frModule.drive.type;
  public static final String kFrontRightSteerType = frModule.angle.type;
  public static final String kFrontRightEncoderType = frModule.encoder.type;
  public static final double kFrontRightEncoderOffset = frModule.absoluteEncoderOffset;
  public static final boolean kFrontRightDriveInvert = frModule.inverted.drive;
  public static final boolean kFrontRightSteerInvert = frModule.inverted.angle;
  public static final boolean kFrontRightEncoderInvert = frModule.absoluteEncoderInverted;
  public static final double kFrontRightXPosInches = frModule.location.left;
  public static final double kFrontRightYPosInches = frModule.location.front;

  // Back Left
  static ModuleJson blModule = moduleJsons[moduleList.indexOf("backleft.json")];
  public static final int kBackLeftDriveMotorId = blModule.drive.id;
  public static final int kBackLeftSteerMotorId = blModule.angle.id;
  public static final int kBackLeftEncoderId = blModule.encoder.id;
  public static final String kBackLeftDriveCanbus = blModule.drive.canbus;
  public static final String kBackLeftSteerCanbus = blModule.angle.canbus;
  public static final String kBackLeftEncoderCanbus = blModule.encoder.canbus;
  public static final String kBackLeftDriveType = blModule.drive.type;
  public static final String kBackLeftSteerType = blModule.angle.type;
  public static final String kBackLeftEncoderType = blModule.encoder.type;
  public static final double kBackLeftEncoderOffset = blModule.absoluteEncoderOffset;
  public static final boolean kBackLeftDriveInvert = blModule.inverted.drive;
  public static final boolean kBackLeftSteerInvert = blModule.inverted.angle;
  public static final boolean kBackLeftEncoderInvert = blModule.absoluteEncoderInverted;
  public static final double kBackLeftXPosInches = blModule.location.left;
  public static final double kBackLeftYPosInches = blModule.location.front;

  // Back Right
  static ModuleJson brModule = moduleJsons[moduleList.indexOf("backright.json")];
  public static final int kBackRightDriveMotorId = brModule.drive.id;
  public static final int kBackRightSteerMotorId = brModule.angle.id;
  public static final int kBackRightEncoderId = brModule.encoder.id;
  public static final String kBackRightDriveCanbus = brModule.drive.canbus;
  public static final String kBackRightSteerCanbus = brModule.angle.canbus;
  public static final String kBackRightEncoderCanbus = brModule.encoder.canbus;
  public static final String kBackRightDriveType = brModule.drive.type;
  public static final String kBackRightSteerType = brModule.angle.type;
  public static final String kBackRightEncoderType = brModule.encoder.type;
  public static final double kBackRightEncoderOffset = brModule.absoluteEncoderOffset;
  public static final boolean kBackRightDriveInvert = brModule.inverted.drive;
  public static final boolean kBackRightSteerInvert = brModule.inverted.angle;
  public static final boolean kBackRightEncoderInvert = brModule.absoluteEncoderInverted;
  public static final double kBackRightXPosInches = brModule.location.left;
  public static final double kBackRightYPosInches = brModule.location.front;

  // YAGSL PIDF Values
  public static final double kDriveP = pidfPropertiesJson.drive.p;
  public static final double kDriveI = pidfPropertiesJson.drive.i;
  public static final double kDriveD = pidfPropertiesJson.drive.d;
  public static final double kDriveF = pidfPropertiesJson.drive.f;
  public static final double kDriveIZ = pidfPropertiesJson.drive.iz;
  public static final double kSteerP = pidfPropertiesJson.angle.p;
  public static final double kSteerI = pidfPropertiesJson.angle.i;
  public static final double kSteerD = pidfPropertiesJson.angle.d;
  public static final double kSteerF = pidfPropertiesJson.angle.f;
  public static final double kSteerIZ = pidfPropertiesJson.angle.iz;
}
