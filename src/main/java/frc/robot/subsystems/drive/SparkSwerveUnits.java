package frc.robot.subsystems.drive;

import frc.robot.util.MathUtil;
import org.wpilib.math.geometry.Rotation2d;

/** Unit conversions for REVLib 2027 SPARK encoders and closed-loop controllers. */
final class SparkSwerveUnits {
  private SparkSwerveUnits() {}

  // REVLib 2027 exposes raw motor rotations/RPM and absolute encoder rotations/RPM.
  static double drivePositionRadians(double motorRotations) {
    return motorRotations * SwerveConstants.driveEncoderPositionFactor;
  }

  static double driveVelocityRadiansPerSecond(double motorRpm) {
    return motorRpm * SwerveConstants.driveEncoderVelocityFactor;
  }

  static double driveVelocityRpm(double wheelRadiansPerSecond) {
    return wheelRadiansPerSecond / SwerveConstants.driveEncoderVelocityFactor;
  }

  static Rotation2d turnPosition(double absoluteRotations, Rotation2d zeroRotation) {
    return Rotation2d.fromRotations(absoluteRotations).minus(zeroRotation);
  }

  static double turnVelocityRadiansPerSecond(double absoluteRpm) {
    return absoluteRpm * SwerveConstants.turnEncoderVelocityFactor;
  }

  static double turnSetpointRotations(Rotation2d rotation, Rotation2d zeroRotation) {
    return MathUtil.inputModulus(rotation.plus(zeroRotation).getRotations(), 0.0, 1.0);
  }

  // Maintain the old response as the controller's input changes from rad/s to RPM.
  static double driveVelocityGain(double gainPerRadianPerSecond) {
    return gainPerRadianPerSecond * SwerveConstants.driveEncoderVelocityFactor;
  }

  // Maintain the old response as the controller's input changes from radians to rotations.
  static double turnPositionGain(double gainPerRadian) {
    return gainPerRadian * SwerveConstants.turnEncoderPositionFactor;
  }
}
