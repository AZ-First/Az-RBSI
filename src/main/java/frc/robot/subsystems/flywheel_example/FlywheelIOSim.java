// Copyright (c) 2024-2026 Az-FIRST
// http://github.com/AZ-First
// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the AdvantageKit-License.md file
// at the root directory of this project.

package frc.robot.subsystems.flywheel_example;

import frc.robot.Constants;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.util.MathUtil;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.LinearSystem;
import org.wpilib.math.system.Models;
import org.wpilib.simulation.FlywheelSim;
import org.wpilib.system.RobotController;

public class FlywheelIOSim implements FlywheelIO {
  private final DCMotor m_gearbox = DCMotor.getNEO(1);
  private final LinearSystem<N1, N1, N1> m_plant =
      Models.flywheelFromPhysicalConstants(
          m_gearbox,
          FlywheelConstants.kSimMomentOfInertiaKgMetersSq,
          FlywheelConstants.kSimGearing);

  private final FlywheelSim sim = new FlywheelSim(m_plant, m_gearbox);
  private PIDController pid = new PIDController(0.0, 0.0, 0.0);
  private SimpleMotorFeedforward ff = new SimpleMotorFeedforward(0.0, 0.0, 0.0);

  private boolean closedLoop = false;
  private double ffVolts = 0.0;
  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    if (closedLoop) {
      appliedVolts =
          MathUtil.clamp(
              pid.calculate(sim.getAngularVelocity()) + ffVolts,
              -FlywheelConstants.kMaxVoltage,
              FlywheelConstants.kMaxVoltage);
      sim.setInputVoltage(appliedVolts);
    }

    sim.update(Constants.kLoopPeriodSecs);

    inputs.positionRad = 0.0;
    inputs.velocityRadPerSec = sim.getAngularVelocity();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = new double[] {sim.getCurrentDraw()};
  }

  @Override
  public void setVoltage(double volts) {
    closedLoop = false;
    appliedVolts = volts;
    sim.setInputVoltage(volts);
  }

  @Override
  public void setPercent(double percent) {
    setVoltage(percent * RobotController.getBatteryVoltage());
  }

  @Override
  public void setVelocity(double velocityRadPerSec) {
    closedLoop = true;
    pid.setSetpoint(velocityRadPerSec);
    this.ffVolts = ff.calculate(velocityRadPerSec);
  }

  @Override
  public void stop() {
    setVoltage(0.0);
  }

  /** Set gain constants -- no kA */
  @Override
  public void configureGains(double kP, double kI, double kD, double kS, double kV) {
    pid.setPID(kP, kI, kD);
    ff.setKs(kS);
    ff.setKv(kV);
    ff.setKa(0.0);
  }

  /** Set gain constants - with kA */
  public void configureGains(double kP, double kI, double kD, double kS, double kV, double kA) {
    pid.setPID(kP, kI, kD);
    ff.setKs(kS);
    ff.setKv(kV);
    ff.setKa(kA);
  }
}
