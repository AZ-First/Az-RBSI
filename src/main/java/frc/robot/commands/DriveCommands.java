// Copyright (c) 2024-2026 Az-FIRST
// http://github.com/AZ-First
// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the AdvantageKit-License.md file
// at the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.SwerveConstants;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public final class DriveCommands {

  private DriveCommands() {}

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command fieldRelativeDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    SlewRateLimiter xLimiter = joystickLimiter();
    SlewRateLimiter yLimiter = joystickLimiter();
    SlewRateLimiter omegaLimiter = joystickLimiter();

    return Commands.run(
            () -> {
              // Get the Linear Velocity & Omega from inputs
              Translation2d linearVelocity =
                  getLinearVelocity(
                      xLimiter.calculate(xSupplier.getAsDouble()),
                      yLimiter.calculate(ySupplier.getAsDouble()));
              double omega = getOmega(omegaLimiter.calculate(omegaSupplier.getAsDouble()));

              // Convert to field relative speeds & send command
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                      linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                      omega * drive.getMaxAngularSpeedRadPerSec());
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getAllianceAwareHeading(drive)));
            },
            drive)
        .beforeStarting(() -> resetLimiters(xLimiter, yLimiter, omegaLimiter));
  }

  /**
   * Robot relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command robotRelativeDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    SlewRateLimiter xLimiter = joystickLimiter();
    SlewRateLimiter yLimiter = joystickLimiter();
    SlewRateLimiter omegaLimiter = joystickLimiter();

    return Commands.run(
            () -> {
              // Get the Linear Velocity & Omega from inputs
              Translation2d linearVelocity =
                  getLinearVelocity(
                      xLimiter.calculate(xSupplier.getAsDouble()),
                      yLimiter.calculate(ySupplier.getAsDouble()));
              double omega = getOmega(omegaLimiter.calculate(omegaSupplier.getAsDouble()));

              // Run with straight-up velocities w.r.t. the robot!
              drive.runVelocity(
                  new ChassisSpeeds(
                      linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                      linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                      omega * drive.getMaxAngularSpeedRadPerSec()));
            },
            drive)
        .beforeStarting(() -> resetLimiters(xLimiter, yLimiter, omegaLimiter));
  }

  /**
   * Field relative drive command using joystick for linear control and PID for angular control.
   * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
   * absolute rotation with a joystick.
   */
  public static Command fieldRelativeDriveAtAngle(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> rotationSupplier) {
    SlewRateLimiter xLimiter = joystickLimiter();
    SlewRateLimiter yLimiter = joystickLimiter();

    // Construct command
    return Commands.run(
            () -> {
              // Get linear velocity
              Translation2d linearVelocity =
                  getLinearVelocity(
                      xLimiter.calculate(xSupplier.getAsDouble()),
                      yLimiter.calculate(ySupplier.getAsDouble()));

              // Calculate angular speed
              double omega =
                  drive
                      .getAngleController()
                      .calculate(
                          drive.getHeading().getRadians(), rotationSupplier.get().getRadians());

              // Convert to field relative speeds & send command
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                      linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                      omega);
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getAllianceAwareHeading(drive)));
            },
            drive)

        // Reset PID controller when command starts & ends;
        .beforeStarting(
            () -> {
              resetLimiters(xLimiter, yLimiter);
              drive.resetHeadingController();
            })
        .finallyDo(() -> drive.resetHeadingController());
  }

  /** Holds the drive stopped while scheduled. */
  public static Command stop(Drive drive) {
    return Commands.run(drive::stop, drive).finallyDo(drive::stop);
  }

  /** Holds the modules in an X pattern while scheduled. */
  public static Command stopWithX(Drive drive) {
    return Commands.run(drive::stopWithX, drive).finallyDo(drive::stop);
  }

  /** Sets drive motor neutral mode. */
  public static Command setBrakeMode(Drive drive, boolean brake) {
    return Commands.runOnce(() -> drive.setMotorBrake(brake), drive).ignoringDisable(true);
  }

  /** Zeros field-relative heading using alliance-aware orientation. */
  public static Command zeroHeadingForAlliance(Drive drive) {
    return Commands.runOnce(drive::zeroHeadingForAlliance, drive).ignoringDisable(true);
  }

  /** Drives robot-relative at a fixed velocity while scheduled. */
  public static Command robotRelativeNudge(
      Drive drive, double vxMetersPerSec, double vyMetersPerSec, double omegaRadPerSec) {
    ChassisSpeeds speeds = new ChassisSpeeds(vxMetersPerSec, vyMetersPerSec, omegaRadPerSec);
    return Commands.run(() -> drive.runVelocity(speeds), drive).finallyDo(drive::stop);
  }

  /** Utility functions needed by commands in this module ****************** */
  /**
   * Compute the new linear velocity from inputs, including applying deadbands and squaring for
   * smoothness. Slew limiting is applied by the command factory before calling this helper.
   */
  static Translation2d getLinearVelocity(double x, double y) {
    // Apply deadband
    double linearMagnitude =
        MathUtil.applyDeadband(Math.hypot(x, y), OperatorConstants.kJoystickDeadband);
    if (linearMagnitude <= 0.0) {
      return Translation2d.kZero;
    }

    // Square magnitude for more precise control
    // NOTE: The x & y values range from -1 to +1, so their squares are as well
    double scaledMagnitude = linearMagnitude * linearMagnitude;
    double inputMagnitude = Math.hypot(x, y);
    double scale = inputMagnitude > 1e-9 ? scaledMagnitude / inputMagnitude : 0.0;

    // Return new linear velocity
    return new Translation2d(x * scale, y * scale);
  }

  /**
   * Compute the new angular velocity from inputs, including applying deadbands and squaring for
   * smoothness. Slew limiting is applied by the command factory before calling this helper.
   */
  static double getOmega(double omega) {
    omega = MathUtil.applyDeadband(omega, OperatorConstants.kJoystickDeadband);
    return Math.copySign(omega * omega, omega);
  }

  private static SlewRateLimiter joystickLimiter() {
    return new SlewRateLimiter(OperatorConstants.kJoystickSlewRateLimit);
  }

  private static void resetLimiters(SlewRateLimiter... limiters) {
    for (SlewRateLimiter limiter : limiters) {
      limiter.reset(0.0);
    }
  }

  private static Rotation2d getAllianceAwareHeading(Drive drive) {
    return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
        ? drive.getHeading().plus(Rotation2d.k180deg)
        : drive.getHeading();
  }

  /***************************************************************************/
  /** DRIVEBASE CHARACTERIZATION COMMANDS ********************************** */
  /**
   * Measures the velocity feedforward constants for the drive motors.
   *
   * <p>This command should only be used in voltage control mode.
   */
  public static Command feedforwardCharacterization(Drive drive) {
    List<Double> velocitySamples = new ArrayList<>();
    List<Double> voltageSamples = new ArrayList<>();
    Timer timer = new Timer();

    return Commands.sequence(
            // Reset data
            Commands.runOnce(
                () -> {
                  velocitySamples.clear();
                  voltageSamples.clear();
                }),

            // Allow modules to orient
            Commands.run(
                    () -> {
                      drive.runCharacterization(0.0);
                    },
                    drive)
                .withTimeout(DrivebaseConstants.kFeedforwardCharacterizationStartDelaySecs),

            // Start timer
            Commands.runOnce(timer::restart),

            // Accelerate and gather data
            Commands.run(
                    () -> {
                      double voltage =
                          timer.get()
                              * DrivebaseConstants.kFeedforwardCharacterizationRampRateVoltsPerSec;
                      drive.runCharacterization(voltage);
                      velocitySamples.add(drive.getFFCharacterizationVelocity());
                      voltageSamples.add(voltage);
                    },
                    drive)

                // When cancelled, calculate and print results
                .finallyDo(
                    () -> {
                      int n = velocitySamples.size();
                      double sumX = 0.0;
                      double sumY = 0.0;
                      double sumXY = 0.0;
                      double sumX2 = 0.0;
                      for (int i = 0; i < n; i++) {
                        sumX += velocitySamples.get(i);
                        sumY += voltageSamples.get(i);
                        sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                        sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                      }
                      double denominator = n * sumX2 - sumX * sumX;
                      if (n < 2 || Math.abs(denominator) < 1e-9) {
                        DriverStation.reportWarning(
                            "Drive FF characterization did not collect enough usable samples.",
                            false);
                        drive.stop();
                        return;
                      }

                      double kS = (sumY * sumX2 - sumX * sumXY) / denominator;
                      double kV = (n * sumXY - sumX * sumY) / denominator;

                      NumberFormat formatter = new DecimalFormat("#0.00000");
                      System.out.println("********** Drive FF Characterization Results **********");
                      System.out.println("\tkS: " + formatter.format(kS));
                      System.out.println("\tkV: " + formatter.format(kV));
                      drive.stop();
                    }))
        .finallyDo(drive::stop);
  }

  /** Measures the robot's wheel radius by spinning in a circle. */
  public static Command wheelRadiusCharacterization(Drive drive) {
    SlewRateLimiter limiter =
        new SlewRateLimiter(DrivebaseConstants.kWheelRadiusCharacterizationRampRateRadPerSecSq);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

    return Commands.parallel(
            // Drive control sequence
            Commands.sequence(
                // Reset acceleration limiter
                Commands.runOnce(
                    () -> {
                      limiter.reset(0.0);
                    }),

                // Turn in place, accelerating up to full speed
                Commands.run(
                    () -> {
                      double speed =
                          limiter.calculate(
                              DrivebaseConstants.kWheelRadiusCharacterizationMaxVelocityRadPerSec);
                      drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                    },
                    drive)),

            // Measurement sequence
            Commands.sequence(
                // Wait for modules to fully orient before starting measurement
                Commands.waitSeconds(DrivebaseConstants.kWheelRadiusCharacterizationStartDelaySecs),

                // Record starting measurement
                Commands.runOnce(
                    () -> {
                      state.positions = drive.getWheelRadiusCharacterizationPositions();
                      state.lastAngle = drive.getHeading();
                      state.gyroDelta = 0.0;
                    }),

                // Update gyro delta
                Commands.run(
                        () -> {
                          var rotation = drive.getHeading();
                          state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                          state.lastAngle = rotation;
                        })

                    // When cancelled, calculate and print results
                    .finallyDo(
                        () -> {
                          double[] positions = drive.getWheelRadiusCharacterizationPositions();
                          double wheelDelta = 0.0;
                          for (int i = 0; i < 4; i++) {
                            wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                          }
                          if (wheelDelta <= 1e-9) {
                            DriverStation.reportWarning(
                                "Wheel radius characterization did not measure wheel movement.",
                                false);
                            drive.stop();
                            return;
                          }

                          double wheelRadius =
                              (state.gyroDelta * SwerveConstants.kDriveBaseRadiusMeters)
                                  / wheelDelta;

                          NumberFormat formatter = new DecimalFormat("#0.000");
                          System.out.println(
                              "********** Wheel Radius Characterization Results **********");
                          System.out.println(
                              "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                          System.out.println(
                              "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                          System.out.println(
                              "\tWheel Radius: "
                                  + formatter.format(wheelRadius)
                                  + " meters, "
                                  + formatter.format(Units.metersToInches(wheelRadius))
                                  + " inches");
                          drive.stop();
                        })))
        .finallyDo(drive::stop);
  }

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = Rotation2d.kZero;
    double gyroDelta = 0.0;
  }
}
