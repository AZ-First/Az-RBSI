package frc.robot.generated;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

public final class COMPBOTTunerView implements TunerView {
  @Override
  public CANBus canBus() {
    return COMPBOTTunerConstants.kCANBus;
  }

  @Override
  public SwerveDrivetrainConstants drivetrain() {
    return COMPBOTTunerConstants.DrivetrainConstants;
  }

  @Override
  public SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      frontLeft() {
    return COMPBOTTunerConstants.FrontLeft;
  }

  @Override
  public SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      frontRight() {
    return COMPBOTTunerConstants.FrontRight;
  }

  @Override
  public SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      backLeft() {
    return COMPBOTTunerConstants.BackLeft;
  }

  @Override
  public SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      backRight() {
    return COMPBOTTunerConstants.BackRight;
  }
}
