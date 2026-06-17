// Copyright (c) 2024-2026 Az-FIRST
// http://github.com/AZ-First
// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the AdvantageKit-License.md file
// at the root directory of this project.

package frc.robot.subsystems.flywheel_example;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.FlywheelConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.util.RBSISubsystem;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

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

    // Configure SysId routines. The voltage routine is the preferred source for kS/kV/kA fits.
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
    io.setPercent(batteryVoltage > 0.0 ? MathUtil.clamp(volts / batteryVoltage, -1.0, 1.0) : 0.0);
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

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdVoltageQuasistatic(direction);
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdVoltageDynamic(direction);
  }

  /** Returns a command to run a direct-voltage quasistatic test. */
  public Command sysIdVoltageQuasistatic(SysIdRoutine.Direction direction) {
    return voltageSysId
        .quasistatic(direction)
        .withName("Flywheel SysId Voltage " + directionLabel(direction) + " Quasistatic");
  }

  /** Returns a command to run a direct-voltage dynamic test. */
  public Command sysIdVoltageDynamic(SysIdRoutine.Direction direction) {
    return voltageSysId
        .dynamic(direction)
        .withName("Flywheel SysId Voltage " + directionLabel(direction) + " Dynamic");
  }

  /** Returns a command to run a duty-cycle quasistatic test. */
  public Command sysIdDutyCycleQuasistatic(SysIdRoutine.Direction direction) {
    return dutyCycleSysId
        .quasistatic(direction)
        .withName("Flywheel SysId Duty Cycle " + directionLabel(direction) + " Quasistatic");
  }

  /** Returns a command to run a duty-cycle dynamic test. */
  public Command sysIdDutyCycleDynamic(SysIdRoutine.Direction direction) {
    return dutyCycleSysId
        .dynamic(direction)
        .withName("Flywheel SysId Duty Cycle " + directionLabel(direction) + " Dynamic");
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

  @Override
  public int[] getPowerPorts() {
    return io.getPowerPorts();
  }

  private SysIdRoutine createSysIdRoutine(String name, java.util.function.DoubleConsumer output) {
    return new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.of(kSysIdQuasistaticRampRateVoltsPerSec).per(Second),
            Volts.of(kSysIdDynamicStepVoltageVolts),
            Seconds.of(kSysIdTimeoutSecs),
            (state) -> recordSysIdState(name, state)),
        new SysIdRoutine.Mechanism(
            (voltage) -> output.accept(voltage.in(Volts)), null, this, name));
  }

  private void recordSysIdState(String routineName, SysIdRoutine.State state) {
    Logger.recordOutput("SysIdTestState", state.toString());
    Logger.recordOutput("Flywheel/SysIdRoutine", routineName);
    Logger.recordOutput("Flywheel/SysIdState", state.toString());
  }

  private static String directionLabel(SysIdRoutine.Direction direction) {
    return direction == SysIdRoutine.Direction.kForward ? "Forward" : "Reverse";
  }
}
