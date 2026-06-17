# RBSI SysId Characterization

This document describes the RBSI SysId routines for the example flywheel and how to use the
resulting data with WPILib SysId, AdvantageKit, CTRE Phoenix 6, and REVLib.

## References

- WPILib SysId routine setup:
  https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/creating-routine.html
- WPILib SysId data loading:
  https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/loading-data.html
- WPILib 2026 `SysIdRoutine.Config` API:
  https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj2/command/sysid/SysIdRoutine.Config.html
- AdvantageKit SysId compatibility:
  https://docs.advantagekit.org/data-flow/sysid-compatibility
- CTRE Phoenix 6 SysId integration:
  https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/wpilib-integration/sysid-integration/plumbing-and-running-sysid.html
- CTRE Phoenix 6 control requests:
  https://v6.docs.ctr-electronics.com/en/latest/docs/migration/migration-guide/control-requests-guide.html
- REVLib Spark configuration:
  https://docs.revrobotics.com/revlib/spark/configuring-a-spark
- REVLib Spark velocity control:
  https://docs.revrobotics.com/revlib/spark/closed-loop/velocity-control-mode
- REVLib `SparkClosedLoopController.setSetpoint` API:
  https://codedocs.revrobotics.com/java/com/revrobotics/spark/sparkclosedloopcontroller

## Implemented Routines

RBSI exposes two SysId routine families for the example flywheel:

- `FlywheelVoltage`: direct voltage control.
- `FlywheelDutyCycle`: duty-cycle control, where the SysId requested voltage is converted to a
  percent output using the current roboRIO battery voltage.

The voltage routine is the primary routine for identifying feedforward constants. It uses the
vendor voltage APIs directly:

- Phoenix 6: `VoltageOut`
- REVLib Spark: `SparkBase.setVoltage`
- Simulation: `FlywheelSim.setInputVoltage`

The duty-cycle routine is useful as a comparison check. It exercises the duty-cycle path:

- Phoenix 6: `DutyCycleOut`
- REVLib Spark: `SparkBase.set`
- Simulation: requested percent times `RobotController.getBatteryVoltage()`

Use duty-cycle results as a sanity check against the direct-voltage fit. If the two fits are very
different, inspect battery sag, current limiting, ramp rates, gearing, follower configuration, and
logged applied voltage before copying constants.

Closed-loop velocity methods are implemented separately for normal mechanism control:

- Phoenix 6 regular velocity: `VelocityVoltage`
- Phoenix 6 profiled velocity: `MotionMagicVelocityVoltage`
- REVLib Spark velocity: `SparkClosedLoopController.setSetpoint(..., ControlType.kVelocity, ...)`

Do not use closed-loop velocity control to generate feedforward fits in WPILib SysId. SysId is
trying to identify the plant from applied voltage, position, and velocity. A closed-loop velocity
controller changes the voltage to chase a speed target, which makes the fit difficult to interpret.
Use SysId to find feedforward constants, then use those constants in the closed-loop velocity
controllers.

## Chooser Entries

The routines are published in the PathPlanner auto chooser when `Constants.getAutoType()` is
`PATHPLANNER`.

Run all four voltage routines:

- `Flywheel SysId Voltage (Quasistatic Forward)`
- `Flywheel SysId Voltage (Quasistatic Reverse)`
- `Flywheel SysId Voltage (Dynamic Forward)`
- `Flywheel SysId Voltage (Dynamic Reverse)`

Optional comparison routines:

- `Flywheel SysId Duty Cycle (Quasistatic Forward)`
- `Flywheel SysId Duty Cycle (Quasistatic Reverse)`
- `Flywheel SysId Duty Cycle (Dynamic Forward)`
- `Flywheel SysId Duty Cycle (Dynamic Reverse)`

## Pre-Run Checklist

1. Verify the flywheel can spin safely in both directions, or temporarily skip reverse tests for a
   mechanism that is not safe in reverse.
2. Put the robot in a safe open area, with the flywheel pointed away from people and loose objects.
3. Use a charged battery.
4. Confirm the example flywheel IO selected in `RobotContainer` matches the hardware being tested.
5. Confirm gear ratio and encoder units in `Constants.FlywheelConstants.kGearRatio`.
6. Confirm the follower motor is physically and logically correct. A missing or inverted follower
   will poison the characterization data.
7. Disable game-piece shooting code, automatic spin-up commands, and other commands that might
   require the flywheel subsystem.

## Running The Tests

Run the voltage family first. The order recommended by WPILib is:

1. Quasistatic Forward
2. Quasistatic Reverse
3. Dynamic Forward
4. Dynamic Reverse

Each routine automatically ends at the configured timeout. RBSI uses:

- quasistatic ramp rate: `Constants.FlywheelConstants.kSysIdQuasistaticRampRateVoltsPerSec`
- dynamic step voltage: `Constants.FlywheelConstants.kSysIdDynamicStepVoltageVolts`
- timeout: `Constants.FlywheelConstants.kSysIdTimeoutSecs`

If the flywheel reaches unsafe speed before timeout, disable immediately and lower the ramp rate,
step voltage, or timeout.

## AdvantageKit Logging Workflow

RBSI follows the AdvantageKit SysId workflow:

- The SysId `Mechanism` log callback is `null`.
- Flywheel inputs are logged through the `FlywheelIOInputsAutoLogged` object.
- The SysId test state is recorded with `Logger.recordOutput("SysIdTestState", ...)`.
- RBSI also records `Flywheel/SysIdRoutine` and `Flywheel/SysIdState` for easier filtering.

Relevant logged fields:

- `Flywheel/PositionRad`
- `Flywheel/VelocityRadPerSec`
- `Flywheel/AppliedVolts`
- `Flywheel/SysIdRoutine`
- `Flywheel/SysIdState`
- `SysIdTestState`

AdvantageKit logs should not be opened directly in WPILib SysId. Convert them first:

1. Open the `.wpilog` in AdvantageScope.
2. Use `File` -> `Export Data...`.
3. Export as `WPILOG`.
4. Use `AdvantageKit Cycles` timestamps.
5. Include the relevant `Flywheel` fields and `SysIdTestState`.
6. Open the exported log in WPILib SysId.

## Loading Data In WPILib SysId

After opening the exported log:

1. Drag `SysIdTestState` into the test-state slot.
2. Drag flywheel position into the position slot.
3. Drag flywheel velocity into the velocity slot.
4. Drag flywheel applied voltage into the voltage slot.
5. Set analysis type to flywheel/angular mechanism.
6. Confirm units:
   - position: radians
   - velocity: radians per second
   - voltage: volts

The WPILib loading documentation notes that normal `SysIdRoutine` logs are named with
`sysid-test-state-<mechanism>`. RBSI uses AdvantageKit's `SysIdTestState` field instead, because
that is the AdvantageKit-compatible state key.

## Applying Results

For the voltage routine, copy the fitted constants into `Constants.FlywheelConstants`:

- `kRealS`
- `kRealV`
- `kRealA`

For simulation-specific fits, update:

- `kSimS`
- `kSimV`
- `kSimA`

Then tune feedback:

- CTRE Phoenix 6: `FlywheelIOTalonFX.configureGains(...)` writes `kP`, `kI`, `kD`, `kS`, `kV`,
  and `kA` into `Slot0`.
- REVLib Spark: `FlywheelIOSpark` configures closed-loop gains in the Spark config object and uses
  WPILib `SimpleMotorFeedforward` as an arbitrary voltage feedforward during velocity control.
- Simulation: `FlywheelIOSim` uses WPILib `PIDController` and `SimpleMotorFeedforward`.

Do feedforward first, then tune `kP` and `kD`. Leave `kI` at zero unless there is a measured,
repeatable steady-state error that feedforward and proportional control cannot address.

## CTRE Notes

Phoenix 6 separates control requests by output type. RBSI uses:

- `VoltageOut` for SysId voltage characterization.
- `DutyCycleOut` for duty-cycle comparison characterization.
- `VelocityVoltage` for normal velocity control.
- `MotionMagicVelocityVoltage` for profiled velocity control.

The TalonFX reports rotor position and velocity in rotations. RBSI converts between motor rotations
and mechanism radians using `kGearRatio`. The velocity setpoint sent to CTRE is motor
rotations per second, so RBSI multiplies mechanism rotations per second by the gear ratio.

If Phoenix Pro is licensed, RBSI enables FOC on supported requests. If it is not licensed, Phoenix 6
falls back for supported FOC requests. Do not use torque-current characterization unless the robot
code and analysis workflow are intentionally changed for that model.

## REVLib Notes

RBSI uses the 2026 Spark configuration API:

- configure once through a `SparkFlexConfig`
- apply with `ResetMode` and `PersistMode`
- use `SparkClosedLoopController.setSetpoint` for velocity control
- pass arbitrary feedforward in volts with `ArbFFUnits.kVoltage`

The Spark encoder velocity is reported in RPM, so RBSI converts mechanism radians per second to
motor RPM for velocity setpoints.

If using SPARK MAX instead of SPARK Flex, use the matching REV configuration class for that device
when adapting this example. The example currently constructs `SparkMax` objects but uses the shared
Spark configuration shape already accepted by the project build.

## Interpreting Bad Data

Common symptoms and likely causes:

- Quasistatic data bends sharply near high voltage: current limit, brownout, mechanical drag, or
  the flywheel hit an unsafe speed.
- Forward and reverse `kS` differ greatly: direction-dependent friction, bad follower inversion, or
  a mechanism that should not be characterized in reverse.
- Dynamic data is noisy: step voltage too high, flywheel too light, loose encoder signal, or
  insufficient logging frequency.
- Duty-cycle fit differs from voltage fit: battery sag or vendor duty-cycle behavior is affecting
  the actual applied voltage.
- Closed-loop velocity overshoots after applying feedforward: reduce `kP`, inspect `kD`, verify the
  feedforward units, and confirm the gear ratio.

## Release Checklist

Before treating constants as final:

1. Run all four voltage routines on the real mechanism.
2. Export the AdvantageKit log using AdvantageKit-cycle timestamps.
3. Fit in WPILib SysId and inspect residuals.
4. Update `Constants.FlywheelConstants`.
5. Rebuild and redeploy.
6. Test `runVelocity(...)` at several RPM setpoints.
7. Only then compare the duty-cycle routine, if desired.
