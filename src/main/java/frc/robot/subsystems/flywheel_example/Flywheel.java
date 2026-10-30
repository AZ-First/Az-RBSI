// Copyright (c) 2024-2027 Az-FIRST
// http://github.com/AZ-First
// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the AdvantageKit-License.md file
// at the root directory of this project.

package frc.robot.subsystems.flywheel_example;

import static frc.robot.Constants.FlywheelConstants.*;
import static org.wpilib.units.Units.Second;
import static org.wpilib.units.Units.Seconds;
import static org.wpilib.units.Units.Volts;

import frc.robot.Constants;
import frc.robot.util.RBSISubsystem;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Command;
import org.wpilib.command3.sysid.SysIdRoutine;
import org.wpilib.math.util.Units;
import org.wpilib.system.RobotController;

public class Flywheel extends RBSISubsystem {
  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

  private final SysIdRoutine voltageSysId;
  private final SysIdRoutine dutyCycleSysId;

  /** Creates a new Flywheel. */
  public Flywheel(FlywheelIO io) {
    this.io = io;

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (Constants.getMode()) {
      case REAL:
      case REPLAY:
        io.configureGains(kRealP, 0.0, kRealD, kRealS, kRealV, kRealA);
        break;
      case SIM:
      default:
        io.configureGains(kSimP, 0.0, kSimD, kSimS, kSimV, kSimA);
        break;
    }

    // Direct voltage is preferred for kS/kV/kA fits; duty cycle is useful for comparison.
    voltageSysId = createSysIdRoutine("FlywheelVoltage", this::runVolts);
    dutyCycleSysId = createSysIdRoutine("FlywheelDutyCycle", this::runDutyCycleForSysIdVolts);
  }

  /** Periodic function -- inherits timing logic from RBSISubsystem */
  @Override
  protected void rbsiPeriodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel", inputs);
  }

  /** Run open loop at the specified voltage. */
  public void runVolts(double volts) {
    io.setVoltage(volts);
  }

  /**
   * Run open loop using duty cycle while treating the SysId output as requested motor voltage.
   *
   * <p>This is useful for comparing vendor duty-cycle behavior against direct voltage control. Use
   * the logged applied voltage, not the requested duty cycle, when fitting SysId data.
   */
  public void runDutyCycleForSysIdVolts(double volts) {
    double batteryVoltage = RobotController.getBatteryVoltage();
    io.setPercent(batteryVoltage > 0.0 ? Math.clamp(volts / batteryVoltage, -1.0, 1.0) : 0.0);
  }

  /** Run closed loop at the specified velocity. */
  public void runVelocity(double velocityRPM) {
    var velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
    io.setVelocity(velocityRadPerSec);

    // Log flywheel setpoint
    Logger.recordOutput("Flywheel/SetpointRPM", velocityRPM);
  }

  /**
   * Run profiled/smoothed closed loop at the specified velocity, when supported by the IO layer.
   */
  public void runVelocityProfiled(double velocityRPM) {
    var velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
    io.setVelocityProfiled(velocityRadPerSec);

    Logger.recordOutput("Flywheel/ProfiledSetpointRPM", velocityRPM);
  }

  /** Stops the flywheel. */
  public void stop() {
    io.stop();
  }

  /** Returns a direct-voltage quasistatic test. */
  public Command sysIdVoltageQuasistatic(SysIdRoutine.Direction direction) {
    return Command.sequence(voltageSysId.quasistatic(direction))
        .named("Flywheel SysId Voltage " + directionLabel(direction) + " Quasistatic");
  }

  /** Returns a direct-voltage dynamic test. */
  public Command sysIdVoltageDynamic(SysIdRoutine.Direction direction) {
    return Command.sequence(voltageSysId.dynamic(direction))
        .named("Flywheel SysId Voltage " + directionLabel(direction) + " Dynamic");
  }

  /** Returns a duty-cycle quasistatic test. */
  public Command sysIdDutyCycleQuasistatic(SysIdRoutine.Direction direction) {
    return Command.sequence(dutyCycleSysId.quasistatic(direction))
        .named("Flywheel SysId Duty Cycle " + directionLabel(direction) + " Quasistatic");
  }

  /** Returns a duty-cycle dynamic test. */
  public Command sysIdDutyCycleDynamic(SysIdRoutine.Direction direction) {
    return Command.sequence(dutyCycleSysId.dynamic(direction))
        .named("Flywheel SysId Duty Cycle " + directionLabel(direction) + " Dynamic");
  }

  /** Returns the current velocity in RPM. */
  @AutoLogOutput(key = "Mechanism/Flywheel")
  public double getVelocityRPM() {
    return Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec);
  }

  /** Returns the current velocity in radians per second. */
  public double getCharacterizationVelocity() {
    return inputs.velocityRadPerSec;
  }

  private SysIdRoutine createSysIdRoutine(String name, java.util.function.DoubleConsumer output) {
    return new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.of(kSysIdQuasistaticRampRateVoltsPerSec).per(Second),
            Volts.of(kSysIdDynamicStepVoltageVolts),
            Seconds.of(kSysIdTimeoutSecs),
            state -> recordSysIdState(name, state)),
        new SysIdRoutine.SysIdMechanism(
            voltage -> output.accept(voltage.in(Volts)), null, this, name));
  }

  private void recordSysIdState(String routineName, SysIdRoutine.State state) {
    Logger.recordOutput("SysIdTestState", state.toString());
    Logger.recordOutput("Flywheel/SysIdRoutine", routineName);
    Logger.recordOutput("Flywheel/SysIdState", state.toString());
  }

  private static String directionLabel(SysIdRoutine.Direction direction) {
    return direction == SysIdRoutine.Direction.FORWARD ? "Forward" : "Reverse";
  }

  @Override
  public int[] getPowerPorts() {
    return io.getPowerPorts();
  }
}
