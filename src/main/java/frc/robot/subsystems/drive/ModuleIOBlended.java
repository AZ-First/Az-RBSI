// Copyright (c) 2024-2026 Az-FIRST
// http://github.com/AZ-First
// Copyright 2024-2025 FRC 2486
// https://github.com/Coconuts2486-FRC
//
// Use of this source code is governed by a BSD
// license that can be found in the AdvantageKit-License.md file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.subsystems.drive.SwerveConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
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
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
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
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.util.PhoenixUtil;
import frc.robot.util.SparkUtil;
import java.util.Queue;
import org.littletonrobotics.junction.Logger;

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

  // Hardware Objects
  private final TalonFX driveTalon;
  private final SparkBase turnSpark;
  private final CANcoder cancoder;
  private final ClosedLoopOutputType m_DriveMotorClosedLoopOutput =
      switch (Constants.getPhoenixPro()) {
        case LICENSED -> ClosedLoopOutputType.TorqueCurrentFOC;
        case UNLICENSED -> ClosedLoopOutputType.Voltage;
      };

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
  private final Debouncer driveConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer turnConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer turnEncoderConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  // Config
  private final TalonFXConfiguration driveConfig = new TalonFXConfiguration();
  private final SparkMaxConfig turnConfig = new SparkMaxConfig();

  // Values used for calculating feedforward from kS, kV, and kA
  private double lastVelocityRotPerSec = 0.0;
  private long lastTimestampNano = System.nanoTime();

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
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveConfig.Slot0 =
        new Slot0Configs()
            .withKP(DrivebaseConstants.kDriveP)
            .withKI(0.0)
            .withKD(DrivebaseConstants.kDriveD)
            .withKS(DrivebaseConstants.kDriveS)
            .withKV(DrivebaseConstants.kDriveV)
            .withKA(DrivebaseConstants.kDriveA);
    driveConfig.Feedback.SensorToMechanismRatio = SwerveConstants.kDriveGearRatio;
    driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = SwerveConstants.kDriveSlipCurrent;
    driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -SwerveConstants.kDriveSlipCurrent;
    driveConfig.CurrentLimits.StatorCurrentLimit = SwerveConstants.kDriveCurrentLimit;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    // Build the OpenLoopRampsConfigs and ClosedLoopRampsConfigs for current smoothing
    OpenLoopRampsConfigs openRamps = new OpenLoopRampsConfigs();
    openRamps.DutyCycleOpenLoopRampPeriod = DrivebaseConstants.kDriveOpenLoopRampPeriod;
    openRamps.VoltageOpenLoopRampPeriod = DrivebaseConstants.kDriveOpenLoopRampPeriod;
    openRamps.TorqueOpenLoopRampPeriod = DrivebaseConstants.kDriveOpenLoopRampPeriod;
    ClosedLoopRampsConfigs closedRamps = new ClosedLoopRampsConfigs();
    closedRamps.DutyCycleClosedLoopRampPeriod = DrivebaseConstants.kDriveClosedLoopRampPeriod;
    closedRamps.VoltageClosedLoopRampPeriod = DrivebaseConstants.kDriveClosedLoopRampPeriod;
    closedRamps.TorqueClosedLoopRampPeriod = DrivebaseConstants.kDriveClosedLoopRampPeriod;
    // Apply the open- and closed-loop ramp configuration for current smoothing
    driveConfig.withClosedLoopRamps(closedRamps).withOpenLoopRamps(openRamps);
    // Set motor inversions
    driveConfig.MotorOutput.Inverted =
        constants.DriveMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    // Configure turn motor
    turnConfig
        .inverted(constants.SteerMotorInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit((int) SwerveConstants.kSteerCurrentLimit)
        .voltageCompensation(DrivebaseConstants.kOptimalVoltage);
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
        .pid(DrivebaseConstants.kSteerP, 0.0, DrivebaseConstants.kSteerD)
        .feedForward
        .kV(0.0);
    turnConfig
        .signals
        .absoluteEncoderPositionAlwaysOn(true)
        .absoluteEncoderPositionPeriodMs((int) (1000.0 / kOdometryFrequency))
        .absoluteEncoderVelocityAlwaysOn(true)
        .absoluteEncoderVelocityPeriodMs((int) (Constants.loopPeriodSecs * 1000.))
        .appliedOutputPeriodMs((int) (Constants.loopPeriodSecs * 1000.))
        .busVoltagePeriodMs((int) (Constants.loopPeriodSecs * 1000.))
        .outputCurrentPeriodMs((int) (Constants.loopPeriodSecs * 1000.));
    turnConfig
        .openLoopRampRate(DrivebaseConstants.kDriveOpenLoopRampPeriod)
        .closedLoopRampRate(DrivebaseConstants.kDriveClosedLoopRampPeriod);

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
    SparkUtil.tryUntilOk(
        turnSpark,
        5,
        () ->
            turnSpark.configure(
                turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
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

  /**
   * Set the drive motor to an open-loop voltage, scaled to battery voltage
   *
   * @param output Specified open-loop voltage requested
   */
  @Override
  public void setDriveOpenLoop(double output) {
    // Scale by actual battery voltage to keep full output consistent
    double busVoltage = RobotController.getBatteryVoltage();
    double scaledOutput = output * DrivebaseConstants.kOptimalVoltage / busVoltage;

    driveTalon.setControl(
        switch (m_DriveMotorClosedLoopOutput) {
          case Voltage -> voltageRequest.withOutput(scaledOutput);
          case TorqueCurrentFOC -> torqueCurrentRequest.withOutput(scaledOutput);
        });

    // Log output and battery
    Logger.recordOutput("Swerve/Drive/OpenLoopOutput", scaledOutput);
    Logger.recordOutput("Robot/BatteryVoltage", busVoltage);
  }

  /**
   * Set the turn motor to an open-loop voltage, scaled to battery voltage
   *
   * @param output Specified open-loop voltage requested
   */
  @Override
  public void setTurnOpenLoop(double output) {
    double busVoltage = RobotController.getBatteryVoltage();
    double scaledOutput = output * DrivebaseConstants.kOptimalVoltage / busVoltage;
    turnSpark.setVoltage(scaledOutput);

    // Log output and battery
    Logger.recordOutput("Swerve/Turn/OpenLoopOutput", scaledOutput);
    Logger.recordOutput("Robot/BatteryVoltage", busVoltage);
  }

  /**
   * Set the velocity of the module
   *
   * @param velocityRadPerSec Requested module drive velocity in radians per second
   */
  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    double velocityRotPerSec = Units.radiansToRotations(velocityRadPerSec);
    double busVoltage = RobotController.getBatteryVoltage();

    // Compute the Feedforward voltage for CTRE UNLICENSED operation *****
    // Compute acceleration
    long currentTimeNano = System.nanoTime();
    double deltaTimeSec = (currentTimeNano - lastTimestampNano) * 1e-9;
    double accelerationRotPerSec2 =
        deltaTimeSec > 0 ? (velocityRotPerSec - lastVelocityRotPerSec) / deltaTimeSec : 0.0;
    // Update last values for next loop
    lastVelocityRotPerSec = velocityRotPerSec;
    lastTimestampNano = currentTimeNano;
    // Compute feedforward voltage: kS + kV*v + kA*a
    double nominalFFVolts =
        Math.signum(velocityRotPerSec) * DrivebaseConstants.kDriveS
            + DrivebaseConstants.kDriveV * velocityRotPerSec
            + DrivebaseConstants.kDriveA * accelerationRotPerSec2;
    double scaledFFVolts = nominalFFVolts * DrivebaseConstants.kOptimalVoltage / busVoltage;

    // Set the drive motor control based on CTRE LICENSED status
    driveTalon.setControl(
        switch (m_DriveMotorClosedLoopOutput) {
          case Voltage ->
              velocityVoltageRequest.withVelocity(velocityRotPerSec).withFeedForward(scaledFFVolts);
          case TorqueCurrentFOC ->
              velocityTorqueCurrentRequest.withVelocity(RotationsPerSecond.of(velocityRotPerSec));
        });

    // AdvantageKit logging
    Logger.recordOutput("Swerve/Drive/VelocityRadPerSec", velocityRadPerSec);
    Logger.recordOutput("Swerve/Drive/VelocityRotPerSec", velocityRotPerSec);
    Logger.recordOutput("Swerve/Drive/AccelerationRotPerSec2", accelerationRotPerSec2);
    Logger.recordOutput("Swerve/Drive/FeedForwardVolts", scaledFFVolts);
    Logger.recordOutput("Robot/BatteryVoltage", busVoltage);
    Logger.recordOutput("Swerve/Drive/ClosedLoopMode", m_DriveMotorClosedLoopOutput);
  }

  /**
   * Set the turn position of the module
   *
   * @param rotation Requested module Rotation2d position
   */
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
