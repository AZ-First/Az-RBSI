// Copyright (c) 2024-2027 Az-FIRST
// http://github.com/AZ-First
// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the AdvantageKit-License.md file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import frc.robot.Constants.DrivebaseConstants;
import java.util.concurrent.atomic.AtomicInteger;
import org.littletonrobotics.junction.Logger;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.math.util.Units;
import org.wpilib.util.Alert;
import org.wpilib.util.Alert.Level;

/**
 * Module API Class
 *
 * <p>The Module class is the API-level class for a single swerve module, of which there are four on
 * the robot. This is not a true subsystem, but an abstraction layer.
 */
public class Module {
  private static final AtomicInteger nextAlertInstance = new AtomicInteger();
  private static final SwerveModulePosition[] EMPTY_ODOMETRY_POSITIONS =
      new SwerveModulePosition[0];

  // Define IO
  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private final int index;

  // Declare alerts here, and only set/unset during the periodic() loop.
  private final Alert driveDisconnectedAlert;
  private final Alert turnDisconnectedAlert;
  private final Alert turnEncoderDisconnectedAlert;
  private SwerveModulePosition[] odometryPositions = EMPTY_ODOMETRY_POSITIONS;

  /** Constructor */
  public Module(ModuleIO io, int index) {
    this.io = io;
    this.index = index;
    int alertInstance = nextAlertInstance.getAndIncrement();
    driveDisconnectedAlert =
        new Alert(
            "module" + index + "DriveDisconnected" + alertInstance,
            "Disconnected drive motor on module " + index + ".",
            Level.HIGH);
    turnDisconnectedAlert =
        new Alert(
            "module" + index + "TurnDisconnected" + alertInstance,
            "Disconnected turn motor on module " + index + ".",
            Level.HIGH);
    turnEncoderDisconnectedAlert =
        new Alert(
            "module" + index + "TurnEncoderDisconnected" + alertInstance,
            "Disconnected turn encoder on module " + index + ".",
            Level.HIGH);
  }

  /************************************************************************* */
  /** Periodic function that is called each robot cycle by the Drive class */
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Drive/Module" + Integer.toString(index), inputs);

    // Calculate positions for odometry
    int sampleCount =
        Math.min(
            inputs.odometryTimestamps.length,
            Math.min(inputs.odometryDrivePositionsRad.length, inputs.odometryTurnPositions.length));
    if (sampleCount <= 0) {
      odometryPositions = EMPTY_ODOMETRY_POSITIONS;
      driveDisconnectedAlert.set(!inputs.driveConnected);
      turnDisconnectedAlert.set(!inputs.turnConnected);
      turnEncoderDisconnectedAlert.set(!inputs.turnEncoderConnected);
      return;
    }
    odometryPositions = new SwerveModulePosition[sampleCount];
    for (int i = 0; i < sampleCount; i++) {
      double positionMeters =
          inputs.odometryDrivePositionsRad[i] * DrivebaseConstants.kWheelRadiusMeters;
      Rotation2d angle = inputs.odometryTurnPositions[i];
      odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
    }

    // Update alerts
    driveDisconnectedAlert.set(!inputs.driveConnected);
    turnDisconnectedAlert.set(!inputs.turnConnected);
    turnEncoderDisconnectedAlert.set(!inputs.turnEncoderConnected);
  }

  /** Returns this module's CTRE telemetry signals for drivetrain-wide refresh. */
  BaseStatusSignal[] getBulkRefreshSignals() {
    return io.getBulkRefreshSignals();
  }

  /** Supplies the result of the drivetrain-wide telemetry refresh. */
  void setBulkRefreshStatus(StatusCode status) {
    io.setBulkRefreshStatus(status);
  }

  /** Forwards the simulation periodic call to the IO layer */
  public void simulationPeriodic() {
    io.simulationPeriodic();
  }

  /** Module Action Functions ********************************************** */
  /**
   * Runs the module with the specified setpoint state
   *
   * <p>Mutates the state to optimize it
   *
   * @param state The requested Swerve Modeule State
   */
  public void runSetpoint(SwerveModuleVelocity state) {
    // Optimize velocity setpoint
    state = state.optimize(getAngle());
    state = state.cosineScale(inputs.turnPosition);

    // Apply setpoints
    io.setDriveVelocity(state.velocity / DrivebaseConstants.kWheelRadiusMeters);
    io.setTurnPosition(state.angle);
  }

  /** Runs the module with the specified output while controlling to zero degrees. */
  public void runCharacterization(double output) {
    io.setDriveOpenLoop(output);
    io.setTurnPosition(Rotation2d.ZERO);
  }

  /** Disables all outputs to motors. */
  public void stop() {
    io.setDriveOpenLoop(0.0);
    io.setTurnOpenLoop(0.0);
  }

  /**
   * Sets whether brake mode is enabled
   *
   * @param enabled Is the brake enabled?
   */
  public void setBrakeMode(boolean enabled) {
    io.setDriveBrakeMode(enabled);
    io.setTurnBrakeMode(enabled);
  }

  /** Getter functions ***************************************************** */
  /** Returns the current turn angle of the module. */
  public Rotation2d getAngle() {
    return inputs.turnPosition;
  }

  /** Returns the current drive position of the module in meters. */
  public double getPositionMeters() {
    return inputs.drivePositionRad * DrivebaseConstants.kWheelRadiusMeters;
  }

  /** Returns the current drive velocity of the module in meters per second. */
  public double getVelocityMetersPerSec() {
    return inputs.driveVelocityRadPerSec * DrivebaseConstants.kWheelRadiusMeters;
  }

  /** Returns the module position (turn angle and drive position). */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  /** Returns the module state (turn angle and drive velocity). */
  public SwerveModuleVelocity getState() {
    return new SwerveModuleVelocity(getVelocityMetersPerSec(), getAngle());
  }

  /** Returns the module positions received this cycle. */
  public SwerveModulePosition[] getOdometryPositions() {
    return odometryPositions;
  }

  /** Returns the timestamps of the samples received this cycle. */
  public double[] getOdometryTimestamps() {
    return inputs.odometryTimestamps;
  }

  /** Returns the module position in radians. */
  public double getWheelRadiusCharacterizationPosition() {
    return inputs.drivePositionRad;
  }

  /** Returns the module velocity in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity() {
    return Units.radiansToRotations(inputs.driveVelocityRadPerSec);
  }
}
