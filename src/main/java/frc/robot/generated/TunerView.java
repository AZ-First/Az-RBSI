package frc.robot.generated;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

public interface TunerView {
  CANBus canBus();

  SwerveDrivetrainConstants drivetrain();

  SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      frontLeft();

  SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      frontRight();

  SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      backLeft();

  SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      backRight();
}
