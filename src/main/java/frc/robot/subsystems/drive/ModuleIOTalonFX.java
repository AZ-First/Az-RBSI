// Copyright (c) 2024-2026 Az-FIRST
// http://github.com/AZ-First
// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
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
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.util.PhoenixUtil;
import java.util.Queue;
import org.littletonrobotics.junction.Logger;

/**
 * Module IO implementation for Talon FX drive motor controller, Talon FX turn motor controller, and
 * CANcoder. Configured using a set of module constants from Phoenix.
 *
 * <p>Device configuration and other behaviors not exposed by TunerConstants can be customized here.
 */
public class ModuleIOTalonFX implements ModuleIO {
  private final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      constants;

  // Hardware objects
  private final TalonFX driveTalon;
  private final TalonFX turnTalon;
  private final CANcoder cancoder;
  private final ClosedLoopOutputType m_DriveMotorClosedLoopOutput =
      switch (Constants.getPhoenixPro()) {
        case LICENSED -> ClosedLoopOutputType.TorqueCurrentFOC;
        case UNLICENSED -> ClosedLoopOutputType.Voltage;
      };
  private final ClosedLoopOutputType m_SteerMotorClosedLoopOutput =
      switch (Constants.getPhoenixPro()) {
        case LICENSED -> ClosedLoopOutputType.TorqueCurrentFOC;
        case UNLICENSED -> ClosedLoopOutputType.Voltage;
      };

  // Voltage control requests
  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final PositionVoltage positionVoltageRequest = new PositionVoltage(0.0);
  private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0.0);

  // Torque-current control requests
  private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0);
  private final PositionTorqueCurrentFOC positionTorqueCurrentRequest =
      new PositionTorqueCurrentFOC(0.0);
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
  private final StatusSignal<Voltage> turnAppliedVolts;
  private final StatusSignal<Current> turnCurrent;

  // Connection debouncers
  private final Debouncer driveConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer turnConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer turnEncoderConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  // Config
  private final TalonFXConfiguration driveConfig = new TalonFXConfiguration();
  private final TalonFXConfiguration turnConfig = new TalonFXConfiguration();

  /*
   * TalonFX I/O
   */
  public ModuleIOTalonFX(int module) {
    constants =
        switch (module) {
          case 0 -> TunerConstants.FrontLeft;
          case 1 -> TunerConstants.FrontRight;
          case 2 -> TunerConstants.BackLeft;
          case 3 -> TunerConstants.BackRight;
          default -> throw new IllegalArgumentException("Invalid module index");
        };

    driveTalon = new TalonFX(constants.DriveMotorId, kCANBus);
    turnTalon = new TalonFX(constants.SteerMotorId, kCANBus);
    cancoder = new CANcoder(constants.EncoderId, kCANBus);

    // Configure drive motor
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveConfig.Slot0 =
        new Slot0Configs()
            .withKP(DrivebaseConstants.kDriveP)
            .withKI(0.0)
            .withKD(DrivebaseConstants.kDriveD)
            .withKS(DrivebaseConstants.kDriveS)
            .withKV(DrivebaseConstants.kDriveV);
    driveConfig.Feedback.SensorToMechanismRatio = SwerveConstants.kDriveGearRatio;
    driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = SwerveConstants.kDriveSlipCurrent;
    driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -SwerveConstants.kDriveSlipCurrent;
    driveConfig.CurrentLimits.StatorCurrentLimit = SwerveConstants.kDriveCurrentLimit;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    driveConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = Constants.loopPeriodSecs;
    driveConfig.MotorOutput.Inverted =
        constants.DriveMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    PhoenixUtil.tryUntilOk(5, () -> driveTalon.getConfigurator().apply(driveConfig, 0.25));
    PhoenixUtil.tryUntilOk(5, () -> driveTalon.setPosition(0.0, 0.25));

    // Configure turn motor
    turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    turnConfig.Slot0 =
        new Slot0Configs()
            .withKP(DrivebaseConstants.kSteerP)
            .withKI(0.0)
            .withKD(DrivebaseConstants.kSteerD)
            .withKS(0.0)
            .withKV(0.0)
            .withKA(0.0)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
    turnConfig.Feedback.FeedbackRemoteSensorID = constants.EncoderId;
    // When not Pro-licensed, FusedCANcoder/SyncCANcoder automatically fall back to RemoteCANcoder
    turnConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    turnConfig.Feedback.RotorToSensorRatio = SwerveConstants.kSteerGearRatio;
    turnConfig.MotionMagic.MotionMagicCruiseVelocity = 100.0 / SwerveConstants.kSteerGearRatio;
    turnConfig.MotionMagic.MotionMagicAcceleration =
        turnConfig.MotionMagic.MotionMagicCruiseVelocity / 0.100;
    turnConfig.MotionMagic.MotionMagicExpo_kV = 0.12 * SwerveConstants.kSteerGearRatio;
    turnConfig.MotionMagic.MotionMagicExpo_kA = 0.1;
    turnConfig.ClosedLoopGeneral.ContinuousWrap = true;
    turnConfig.MotorOutput.Inverted =
        constants.SteerMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    PhoenixUtil.tryUntilOk(5, () -> turnTalon.getConfigurator().apply(turnConfig, 0.25));

    // Configure CANCoder
    CANcoderConfiguration cancoderConfig = constants.EncoderInitialConfigs;
    cancoderConfig.MagnetSensor.MagnetOffset = constants.EncoderOffset;
    cancoderConfig.MagnetSensor.SensorDirection =
        constants.EncoderInverted
            ? SensorDirectionValue.Clockwise_Positive
            : SensorDirectionValue.CounterClockwise_Positive;
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
    turnAbsolutePosition = cancoder.getAbsolutePosition();
    turnPosition = turnTalon.getPosition();
    turnPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(turnTalon.getPosition());
    turnVelocity = turnTalon.getVelocity();
    turnAppliedVolts = turnTalon.getMotorVoltage();
    turnCurrent = turnTalon.getStatorCurrent();

    // Configure periodic frames
    BaseStatusSignal.setUpdateFrequencyForAll(
        SwerveConstants.kOdometryFrequency, drivePosition, turnPosition);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        driveVelocity,
        driveAppliedVolts,
        driveCurrent,
        turnAbsolutePosition,
        turnVelocity,
        turnAppliedVolts,
        turnCurrent);
    ParentDevice.optimizeBusUtilizationForAll(driveTalon, turnTalon);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Refresh all signals
    var driveStatus =
        BaseStatusSignal.refreshAll(drivePosition, driveVelocity, driveAppliedVolts, driveCurrent);
    var turnStatus =
        BaseStatusSignal.refreshAll(turnPosition, turnVelocity, turnAppliedVolts, turnCurrent);
    var turnEncoderStatus = BaseStatusSignal.refreshAll(turnAbsolutePosition);

    // Update drive inputs
    inputs.driveConnected = driveConnectedDebounce.calculate(driveStatus.isOK());
    inputs.drivePositionRad = Units.rotationsToRadians(drivePosition.getValueAsDouble());
    inputs.driveVelocityRadPerSec = Units.rotationsToRadians(driveVelocity.getValueAsDouble());
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveCurrentAmps = driveCurrent.getValueAsDouble();

    // Update turn inputs
    inputs.turnConnected = turnConnectedDebounce.calculate(turnStatus.isOK());
    inputs.turnEncoderConnected = turnEncoderConnectedDebounce.calculate(turnEncoderStatus.isOK());
    inputs.turnAbsolutePosition = Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble());
    inputs.turnPosition = Rotation2d.fromRotations(turnPosition.getValueAsDouble());
    inputs.turnVelocityRadPerSec = Units.rotationsToRadians(turnVelocity.getValueAsDouble());
    inputs.turnAppliedVolts = turnAppliedVolts.getValueAsDouble();
    inputs.turnCurrentAmps = turnCurrent.getValueAsDouble();

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
        switch (m_DriveMotorClosedLoopOutput) {
          case Voltage -> voltageRequest.withOutput(output);
          case TorqueCurrentFOC -> torqueCurrentRequest.withOutput(output);
        });
  }

  @Override
  public void setTurnOpenLoop(double output) {
    turnTalon.setControl(
        switch (m_SteerMotorClosedLoopOutput) {
          case Voltage -> voltageRequest.withOutput(output);
          case TorqueCurrentFOC -> torqueCurrentRequest.withOutput(output);
        });
  }

  /**
   * Set the velocity of the module
   *
   * @param velocityRadPerSec Requested module drive velocity in radians per second
   */
  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    double velocityRotPerSec = Units.radiansToRotations(velocityRadPerSec);
    Logger.recordOutput("DriveBaseTuning/closedloop", m_DriveMotorClosedLoopOutput);
    Logger.recordOutput("DriveBaseTuning/wheelradius_inches", SwerveConstants.kWheelRadiusInches);
    Logger.recordOutput("DriveBaseTuning/talon_vradpersec", velocityRadPerSec);
    Logger.recordOutput("DriveBaseTuning/talon_vrotpersec", velocityRotPerSec);
    Logger.recordOutput("DriveBaseTuning/kDriveP", driveConfig.Slot0.kP);
    Logger.recordOutput("DriveBaseTuning/kDriveD", driveConfig.Slot0.kD);
    Logger.recordOutput("DriveBaseTuning/kSteerP", turnConfig.Slot0.kP);
    Logger.recordOutput("DriveBaseTuning/kSteerD", turnConfig.Slot0.kD);

    driveTalon.setControl(
        switch (m_DriveMotorClosedLoopOutput) {
          case Voltage -> velocityVoltageRequest.withVelocity(velocityRotPerSec);
          case TorqueCurrentFOC ->
              velocityTorqueCurrentRequest.withVelocity(RotationsPerSecond.of(velocityRotPerSec));
        });
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    turnTalon.setControl(
        switch (m_SteerMotorClosedLoopOutput) {
          case Voltage -> positionVoltageRequest.withPosition(rotation.getRotations());
          case TorqueCurrentFOC ->
              positionTorqueCurrentRequest.withPosition(rotation.getRotations());
        });
  }

  @Override
  public void setDrivePID(double kP, double kI, double kD) {
    driveConfig.Slot0.kP = kP;
    driveConfig.Slot0.kI = kI;
    driveConfig.Slot0.kD = kD;
    PhoenixUtil.tryUntilOk(5, () -> driveTalon.getConfigurator().apply(driveConfig, 0.25));
  }

  @Override
  public void setTurnPID(double kP, double kI, double kD) {
    turnConfig.Slot0.kP = kP;
    turnConfig.Slot0.kI = kI;
    turnConfig.Slot0.kD = kD;
    PhoenixUtil.tryUntilOk(5, () -> turnTalon.getConfigurator().apply(turnConfig, 0.25));
  }
}
