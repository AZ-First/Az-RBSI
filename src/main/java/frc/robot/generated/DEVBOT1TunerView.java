package frc.robot.generated;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

public final class DEVBOT1TunerView implements TunerView {
  @Override
  public CANBus canBus() {
    return DEVBOT1TunerConstants.kCANBus;
  }

  @Override
  public SwerveDrivetrainConstants drivetrain() {
    return DEVBOT1TunerConstants.DrivetrainConstants;
  }

  @Override
  public SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      frontLeft() {
    return DEVBOT1TunerConstants.FrontLeft;
  }

  @Override
  public SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      frontRight() {
    return DEVBOT1TunerConstants.FrontRight;
  }

  @Override
  public SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      backLeft() {
    return DEVBOT1TunerConstants.BackLeft;
  }

  @Override
  public SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      backRight() {
    return DEVBOT1TunerConstants.BackRight;
  }
}
