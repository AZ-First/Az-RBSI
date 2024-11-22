package frc.robot.generated;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.swervedrive_ignore.SwerveSubsystem;

// Generated by the Tuner X Swerve Project Generator
// https://v6.docs.ctr-electronics.com/en/stable/docs/tuner/tuner-swerve/index.html
public class TunerConstants {
  // Both sets of gains need to be tuned to your individual robot.

  // The steer motor uses any SwerveModule.SteerRequestType control request with the
  // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
  public static final Slot0Configs steerGains =
      new Slot0Configs().withKP(100).withKI(0).withKD(0.2).withKS(0).withKV(1.5).withKA(0);
  // When using closed-loop control, the drive motor uses the control
  // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
  public static final Slot0Configs driveGains =
      new Slot0Configs().withKP(3).withKI(0).withKD(0).withKS(0).withKV(0).withKA(0);

  // The closed-loop output type to use for the steer motors;
  // This affects the PID/FF gains for the steer motors
  public static final ClosedLoopOutputType steerClosedLoopOutput =
      ClosedLoopOutputType.TorqueCurrentFOC;
  // The closed-loop output type to use for the drive motors;
  // This affects the PID/FF gains for the drive motors
  public static final ClosedLoopOutputType driveClosedLoopOutput =
      ClosedLoopOutputType.TorqueCurrentFOC;

  // The stator current at which the wheels start to slip;
  // This needs to be tuned to your individual robot
  public static final double kSlipCurrentA = 150.0;

  // Initial configs for the drive and steer motors and the CANcoder; these cannot be null.
  // Some configs will be overwritten; check the `with*InitialConfigs()` API documentation.
  public static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();
  public static final TalonFXConfiguration steerInitialConfigs =
      new TalonFXConfiguration()
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  // Swerve azimuth does not require much torque output, so we can set a relatively
                  // low
                  // stator current limit to help avoid brownouts without impacting performance.
                  .withStatorCurrentLimit(60)
                  .withStatorCurrentLimitEnable(true));
  public static final CANcoderConfiguration cancoderInitialConfigs = new CANcoderConfiguration();
  // Configs for the Pigeon 2; leave this null to skip applying Pigeon 2 configs
  public static final Pigeon2Configuration pigeonConfigs = null;

  // Theoretical free speed (m/s) at 12v applied output;
  // This needs to be tuned to your individual robot
  public static final double kSpeedAt12VoltsMps = 5.21;

  // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
  // This may need to be tuned to your individual robot
  public static final double kCoupleRatio = 3.5714285714285716;

  public static final double kDriveGearRatio = 6.122448979591837;
  public static final double kSteerGearRatio = 21.428571428571427;
  public static final double kWheelRadiusInches = 2;

  public static final boolean kInvertLeftSide = false;
  public static final boolean kInvertRightSide = true;

  public static final String kCANbusName = "DriveTrain";
  public static final int kPigeonId = 13;

  // These are only used for simulation
  public static final double kSteerInertia = 0.004;
  public static final double kDriveInertia = 0.025;
  // Simulated voltage necessary to overcome friction
  public static final double kSteerFrictionVoltage = 0.25;
  public static final double kDriveFrictionVoltage = 0.25;

  public static final SwerveDrivetrainConstants DrivetrainConstants =
      new SwerveDrivetrainConstants()
          .withCANbusName(kCANbusName)
          .withPigeon2Id(kPigeonId)
          .withPigeon2Configs(pigeonConfigs);

  public static final SwerveModuleConstantsFactory ConstantCreator =
      new SwerveModuleConstantsFactory()
          .withDriveMotorGearRatio(kDriveGearRatio)
          .withSteerMotorGearRatio(kSteerGearRatio)
          .withWheelRadius(kWheelRadiusInches)
          .withSlipCurrent(kSlipCurrentA)
          .withSteerMotorGains(steerGains)
          .withDriveMotorGains(driveGains)
          .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
          .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
          .withSpeedAt12VoltsMps(kSpeedAt12VoltsMps)
          .withSteerInertia(kSteerInertia)
          .withDriveInertia(kDriveInertia)
          .withSteerFrictionVoltage(kSteerFrictionVoltage)
          .withDriveFrictionVoltage(kDriveFrictionVoltage)
          .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
          .withCouplingGearRatio(kCoupleRatio)
          .withDriveMotorInitialConfigs(driveInitialConfigs)
          .withSteerMotorInitialConfigs(steerInitialConfigs)
          .withCANcoderInitialConfigs(cancoderInitialConfigs);

  // Front Left
  public static final int kFrontLeftDriveMotorId = 1;
  public static final int kFrontLeftSteerMotorId = 2;
  public static final int kFrontLeftEncoderId = 3;
  public static final double kFrontLeftEncoderOffset = -0.303466796875;
  public static final boolean kFrontLeftSteerInvert = true;

  public static final double kFrontLeftXPosInches = 10.375;
  public static final double kFrontLeftYPosInches = 10.375;

  // Front Right
  public static final int kFrontRightDriveMotorId = 4;
  public static final int kFrontRightSteerMotorId = 5;
  public static final int kFrontRightEncoderId = 6;
  public static final double kFrontRightEncoderOffset = -0.122802734375;
  public static final boolean kFrontRightSteerInvert = true;

  public static final double kFrontRightXPosInches = 10.375;
  public static final double kFrontRightYPosInches = -10.375;

  // Back Left
  public static final int kBackLeftDriveMotorId = 7;
  public static final int kBackLeftSteerMotorId = 8;
  public static final int kBackLeftEncoderId = 9;
  public static final double kBackLeftEncoderOffset = 0.4814453125;
  public static final boolean kBackLeftSteerInvert = true;

  public static final double kBackLeftXPosInches = -10.375;
  public static final double kBackLeftYPosInches = 10.375;

  // Back Right
  public static final int kBackRightDriveMotorId = 10;
  public static final int kBackRightSteerMotorId = 11;
  public static final int kBackRightEncoderId = 12;
  public static final double kBackRightEncoderOffset = 0.300537109375;
  public static final boolean kBackRightSteerInvert = true;

  public static final double kBackRightXPosInches = -10.375;
  public static final double kBackRightYPosInches = -10.375;

  public static final SwerveModuleConstants FrontLeft =
      ConstantCreator.createModuleConstants(
              kFrontLeftSteerMotorId,
              kFrontLeftDriveMotorId,
              kFrontLeftEncoderId,
              kFrontLeftEncoderOffset,
              Units.inchesToMeters(kFrontLeftXPosInches),
              Units.inchesToMeters(kFrontLeftYPosInches),
              kInvertLeftSide)
          .withSteerMotorInverted(kFrontLeftSteerInvert);
  public static final SwerveModuleConstants FrontRight =
      ConstantCreator.createModuleConstants(
              kFrontRightSteerMotorId,
              kFrontRightDriveMotorId,
              kFrontRightEncoderId,
              kFrontRightEncoderOffset,
              Units.inchesToMeters(kFrontRightXPosInches),
              Units.inchesToMeters(kFrontRightYPosInches),
              kInvertRightSide)
          .withSteerMotorInverted(kFrontRightSteerInvert);
  public static final SwerveModuleConstants BackLeft =
      ConstantCreator.createModuleConstants(
              kBackLeftSteerMotorId,
              kBackLeftDriveMotorId,
              kBackLeftEncoderId,
              kBackLeftEncoderOffset,
              Units.inchesToMeters(kBackLeftXPosInches),
              Units.inchesToMeters(kBackLeftYPosInches),
              kInvertLeftSide)
          .withSteerMotorInverted(kBackLeftSteerInvert);
  public static final SwerveModuleConstants BackRight =
      ConstantCreator.createModuleConstants(
              kBackRightSteerMotorId,
              kBackRightDriveMotorId,
              kBackRightEncoderId,
              kBackRightEncoderOffset,
              Units.inchesToMeters(kBackRightXPosInches),
              Units.inchesToMeters(kBackRightYPosInches),
              kInvertRightSide)
          .withSteerMotorInverted(kBackRightSteerInvert);

  public static final SwerveSubsystem DriveTrain =
      new SwerveSubsystem(DrivetrainConstants, FrontLeft, FrontRight, BackLeft, BackRight);
}
