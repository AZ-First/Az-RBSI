// Copyright (c) 2024-2026 Az-FIRST
// http://github.com/AZ-First
// Copyright 2024-2025 FRC 2486
// https://github.com/Coconuts2486-FRC
//
// Use of this source code is governed by a BSD
// license that can be found in the AdvantageKit-License.md file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.SwerveConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.util.PhoenixUtil;
import frc.robot.util.SparkUtil;
import java.util.Queue;

/**
 * Module IO implementation for blended TalonFX drive motor controller, SparkMax turn motor
 * controller (NEO or NEO 550), and CANcoder.
 *
 * <p>To calibrate the absolute encoder offsets, point the modules straight (such that forward
 * motion on the drive motor will propel the robot forward) and copy the reported values from the
 * absolute encoders using AdvantageScope. These values are logged under
 * "/Drive/ModuleX/TurnAbsolutePositionRad"
 */
public class ModuleIOBlended implements ModuleIO {
  private final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      constants;

  // CAN Devices
  private final TalonFX driveTalon;
  private final SparkBase turnSpark;
  private final CANcoder cancoder;

  // Closed loop controllers
  private final SparkClosedLoopController turnController;

  // Voltage control requests
  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0.0);

  // Torque-current control requests
  private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0);
  private final VelocityTorqueCurrentFOC velocityTorqueCurrentRequest =
      new VelocityTorqueCurrentFOC(0.0);

  // Timestamp inputs from Phoenix thread
  private final Queue<Double> timestampQueue;

  // Inputs from drive motor
  private final StatusSignal<Angle> drivePosition;
  private final Queue<Double> drivePositionQueue;
  private final StatusSignal<AngularVelocity> driveVelocity;
  private final StatusSignal<Voltage> driveAppliedVolts;
  private final StatusSignal<Current> driveCurrent;

  // Inputs from turn motor
  private final StatusSignal<Angle> turnAbsolutePosition;
  private final StatusSignal<Angle> turnPosition;
  private final Queue<Double> turnPositionQueue;
  private final StatusSignal<AngularVelocity> turnVelocity;

  // Connection debouncers
  private final Debouncer driveConnectedDebounce = new Debouncer(0.5);
  private final Debouncer turnConnectedDebounce = new Debouncer(0.5);
  private final Debouncer turnEncoderConnectedDebounce = new Debouncer(0.5);

  /*
   * Blended Module I/O, using Falcon drive and NEO steer motors
   * Based on the ModuleIOTalonFX module, with the SparkMax components
   * added in appropriately.
   */
  public ModuleIOBlended(int module) {

    constants =
        switch (module) {
          case 0 ->
              ConstantCreator.createModuleConstants(
                  kFLSteerMotorId,
                  kFLDriveMotorId,
                  kFLEncoderId,
                  kFLEncoderOffset,
                  kFLXPosMeters,
                  kFLYPosMeters,
                  kFLDriveInvert,
                  kFLSteerInvert,
                  kFLEncoderInvert);
          case 1 ->
              ConstantCreator.createModuleConstants(
                  kFRSteerMotorId,
                  kFRDriveMotorId,
                  kFREncoderId,
                  kFREncoderOffset,
                  kFRXPosMeters,
                  kFRYPosMeters,
                  kFRDriveInvert,
                  kFRSteerInvert,
                  kFREncoderInvert);
          case 2 ->
              ConstantCreator.createModuleConstants(
                  kBLSteerMotorId,
                  kBLDriveMotorId,
                  kBLEncoderId,
                  kBLEncoderOffset,
                  kBLXPosMeters,
                  kBLYPosMeters,
                  kBLDriveInvert,
                  kBLSteerInvert,
                  kBLEncoderInvert);
          case 3 ->
              ConstantCreator.createModuleConstants(
                  kBRSteerMotorId,
                  kBRDriveMotorId,
                  kBREncoderId,
                  kBREncoderOffset,
                  kBRXPosMeters,
                  kBRYPosMeters,
                  kBRDriveInvert,
                  kBRSteerInvert,
                  kBREncoderInvert);
          default -> throw new IllegalArgumentException("Invalid module index");
        };

    driveTalon = new TalonFX(constants.DriveMotorId, kCANBus);
    turnSpark = new SparkMax(constants.SteerMotorId, MotorType.kBrushless);
    cancoder = new CANcoder(constants.EncoderId, kCANBus);

    turnController = turnSpark.getClosedLoopController();

    // Configure drive motor
    var driveConfig = constants.DriveMotorInitialConfigs;
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveConfig.Slot0 = constants.DriveMotorGains;
    driveConfig.Feedback.SensorToMechanismRatio = SwerveConstants.kDriveGearRatio;
    driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = SwerveConstants.kDriveSlipCurrent;
    driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -SwerveConstants.kDriveSlipCurrent;
    driveConfig.CurrentLimits.StatorCurrentLimit = SwerveConstants.kDriveCurrentLimit;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    driveConfig.MotorOutput.Inverted =
        constants.DriveMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    // Configure turn motor
    var turnConfig = new SparkMaxConfig();
    turnConfig
        .inverted(constants.SteerMotorInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit((int) SwerveConstants.kSteerCurrentLimit)
        .voltageCompensation(12.0);
    turnConfig
        .absoluteEncoder
        .inverted(constants.EncoderInverted)
        .positionConversionFactor(turnEncoderPositionFactor)
        .velocityConversionFactor(turnEncoderVelocityFactor)
        .averageDepth(2);
    turnConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(turnPIDMinInput, turnPIDMaxInput)
        .pid(
            AutoConstants.kPPsteerPID.kP,
            AutoConstants.kPPsteerPID.kI,
            AutoConstants.kPPsteerPID.kD)
        .feedForward
        .kV(0.0);
    turnConfig
        .signals
        .absoluteEncoderPositionAlwaysOn(true)
        .absoluteEncoderPositionPeriodMs((int) (1000.0 / kOdometryFrequency))
        .absoluteEncoderVelocityAlwaysOn(true)
        .absoluteEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    SparkUtil.tryUntilOk(
        turnSpark,
        5,
        () ->
            turnSpark.configure(
                turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    // Configure CANCoder
    CANcoderConfiguration cancoderConfig = constants.EncoderInitialConfigs;
    cancoderConfig.MagnetSensor.MagnetOffset = constants.EncoderOffset;
    cancoderConfig.MagnetSensor.SensorDirection =
        constants.EncoderInverted
            ? SensorDirectionValue.Clockwise_Positive
            : SensorDirectionValue.CounterClockwise_Positive;

    // Set motor Closed Loop Output type based on Phoenix Pro status
    constants.DriveMotorClosedLoopOutput =
        switch (Constants.getPhoenixPro()) {
          case LICENSED -> ClosedLoopOutputType.TorqueCurrentFOC;
          case UNLICENSED -> ClosedLoopOutputType.Voltage;
        };

    // Finally, apply the configs to the motor controllers
    PhoenixUtil.tryUntilOk(5, () -> driveTalon.getConfigurator().apply(driveConfig, 0.25));
    PhoenixUtil.tryUntilOk(5, () -> driveTalon.setPosition(0.0, 0.25));
    cancoder.getConfigurator().apply(cancoderConfig);

    // Create timestamp queue
    timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();

    // Create drive status signals
    drivePosition = driveTalon.getPosition();
    drivePositionQueue =
        PhoenixOdometryThread.getInstance().registerSignal(driveTalon.getPosition());
    driveVelocity = driveTalon.getVelocity();
    driveAppliedVolts = driveTalon.getMotorVoltage();
    driveCurrent = driveTalon.getStatorCurrent();

    // Create turn status signals
    turnVelocity = cancoder.getVelocity();
    turnAbsolutePosition = cancoder.getAbsolutePosition();
    turnPosition = cancoder.getPosition();
    turnPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(cancoder.getPosition());

    // Configure periodic frames
    BaseStatusSignal.setUpdateFrequencyForAll(SwerveConstants.kOdometryFrequency, drivePosition);
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, driveVelocity, driveAppliedVolts, driveCurrent);
    ParentDevice.optimizeBusUtilizationForAll(driveTalon);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Refresh all signals
    var driveStatus =
        BaseStatusSignal.refreshAll(drivePosition, driveVelocity, driveAppliedVolts, driveCurrent);
    var turnEncoderStatus = BaseStatusSignal.refreshAll(turnAbsolutePosition);

    // Update drive inputs
    inputs.driveConnected = driveConnectedDebounce.calculate(driveStatus.isOK());
    inputs.drivePositionRad = Units.rotationsToRadians(drivePosition.getValueAsDouble());
    inputs.driveVelocityRadPerSec = Units.rotationsToRadians(driveVelocity.getValueAsDouble());
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveCurrentAmps = driveCurrent.getValueAsDouble();

    // Update turn inputs
    SparkUtil.sparkStickyFault = false;
    inputs.turnEncoderConnected = turnEncoderConnectedDebounce.calculate(turnEncoderStatus.isOK());
    inputs.turnAbsolutePosition = Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble());
    inputs.turnPosition = Rotation2d.fromRotations(turnPosition.getValueAsDouble());
    inputs.turnVelocityRadPerSec = Units.rotationsToRadians(turnVelocity.getValueAsDouble());
    SparkUtil.ifOk(
        turnSpark, turnSpark::getOutputCurrent, (value) -> inputs.turnCurrentAmps = value);
    inputs.turnConnected = turnConnectedDebounce.calculate(!SparkUtil.sparkStickyFault);

    // Update odometry inputs
    inputs.odometryTimestamps =
        timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsRad =
        drivePositionQueue.stream()
            .mapToDouble((Double value) -> Units.rotationsToRadians(value))
            .toArray();
    inputs.odometryTurnPositions =
        turnPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromRotations(value))
            .toArray(Rotation2d[]::new);
    timestampQueue.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();
  }

  @Override
  public void setDriveOpenLoop(double output) {
    driveTalon.setControl(
        switch (constants.DriveMotorClosedLoopOutput) {
          case Voltage -> voltageRequest.withOutput(output);
          case TorqueCurrentFOC -> torqueCurrentRequest.withOutput(output);
        });
  }

  @Override
  public void setTurnOpenLoop(double output) {
    turnSpark.setVoltage(output);
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    double velocityRotPerSec = Units.radiansToRotations(velocityRadPerSec);
    driveTalon.setControl(
        switch (constants.DriveMotorClosedLoopOutput) {
          case Voltage -> velocityVoltageRequest.withVelocity(velocityRotPerSec);
          case TorqueCurrentFOC -> velocityTorqueCurrentRequest.withVelocity(velocityRotPerSec);
        });
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    double setpoint =
        MathUtil.inputModulus(
            rotation.plus(Rotation2d.fromRotations(constants.EncoderOffset)).getRadians(),
            turnPIDMinInput,
            turnPIDMaxInput);
    turnController.setSetpoint(setpoint, ControlType.kPosition);
  }

  private SwerveModuleConstantsFactory<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      ConstantCreator =
          new SwerveModuleConstantsFactory<
                  TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
              .withDriveMotorGearRatio(kDriveGearRatio)
              .withSteerMotorGearRatio(kSteerGearRatio)
              .withCouplingGearRatio(kCoupleRatio)
              .withWheelRadius(kWheelRadiusMeters)
              .withSteerInertia(kSteerInertia)
              .withDriveInertia(kDriveInertia)
              .withSteerFrictionVoltage(kSteerFrictionVoltage)
              .withDriveFrictionVoltage(kDriveFrictionVoltage);
}
