# RBSI Constants Guide

This document describes `src/main/java/frc/robot/Constants.java`, what each section controls, and
which sections a team should tune first when adapting RBSI to a new robot.

`Constants.java` is intended to be a robot configuration surface, not a logic file. New projects
should keep robot-specific values here, keep behavior in subsystems and commands, and prefer names
that include units when the Java type does not already carry units.

## Naming Pattern

The constants file now follows these conventions:

- Every public constant starts with `k`.
- Names avoid repeating the section name. For example, inside `FlywheelConstants`, use `kGearRatio`
  instead of `kFlywheelGearRatio`.
- Plain `double` values include units when practical, such as `kMaxLinearSpeedMetersPerSec`,
  `kSlipCurrentAmps`, and `kWheelLockTimeSecs`.
- Vendor or WPILib unit-typed values may use shorter names because their type carries the unit.
- Old compatibility aliases have intentionally been removed. This is a template, so new projects
  should learn the clean names from the beginning.

## Robot Selection

Top-level private fields select the active robot and feature stack:

- `robotType`
- `swerveType`
- `phoenixPro`
- `autoType`
- `visionType`

Teams should check these before deploying. The selected `robotType` determines `REAL`, `REPLAY`, or
`SIM` mode through `getMode()`. Do not deploy with `SIMBOT`; the `main(...)` guard rejects it.

Most teams should tune these immediately:

- `robotType`: select the physical robot.
- `swerveType`: `PHOENIX6` or `YAGSL`.
- `phoenixPro`: match the CTRE license status.
- `autoType`: choose `MANUAL`, `PATHPLANNER`, or `CHOREO`.
- `visionType`: choose `PHOTON`, `LIMELIGHT`, or `NONE`.

## General Constants

General constants are robot-wide support values:

- `kLoopPeriodSecs`: main robot loop period.
- `kTuningMode`: enables live tunable-number behavior.
- `kGravityMetersPerSecSq`: conversion from g to meters per second squared.

Most teams should leave these alone unless they have a specific framework-level reason.

## RobotConstants

`RobotConstants` contains physical robot properties used by simulation, path planning, and drive
models:

- `kMass`
- `kChassisMatter`
- `kMomentOfInertiaKgMetersSq`
- `kWheelCoefficientOfFriction`
- `kMaxWheelTorqueNm`
- `kRioOrientation`
- `kIMUOrientation`

Tune these carefully:

- `kMass`: measured robot weight including battery and bumpers.
- `kMomentOfInertiaKgMetersSq`: estimate from CAD or measurement.
- `kWheelCoefficientOfFriction`: carpet/wheel traction estimate.
- `kRioOrientation` and `kIMUOrientation`: must match the physical mounting orientation.

Incorrect orientation constants can corrupt acceleration logging, heading interpretation, and any
logic that rotates sensor values into robot coordinates.

## PowerConstants

`PowerConstants` configures the power distribution device and current/voltage alert thresholds:

- `kPdmType`
- `kPdmCanId`
- `kTotalMaxCurrentAmps`
- `kMotorPortMaxCurrentAmps`
- `kSmallPortMaxCurrentAmps`
- `kWarningVoltage`
- `kLimitingVoltage`
- `kCriticalVoltage`

Tune these early for the real robot:

- `kPdmType` and `kPdmCanId`: must match the installed PDH/PDP.
- Current limits: match breaker, wiring, and mechanism expectations.
- Voltage thresholds: tune alert levels for your team’s brownout policy.

## CANBuses

`CANBuses` names the robot CAN networks:

- `RIO`
- `DRIVE`
- `ALL`

These names must match CTRE, REV, and generated swerve configuration expectations. Add any
additional CANivore or vendor buses here and include them in `ALL` so health monitoring sees them.

## RobotDevices

`RobotDevices` maps mechanisms to CAN IDs, CAN bus names, and power distribution ports.

Drivetrain IDs are sourced from `SwerveConstants`, while power ports are assigned here. Mechanism
devices, such as the example flywheel leader and follower, are also assigned here.

Teams should tune this section carefully:

- Verify every CAN ID.
- Verify every bus name.
- Verify every PDH/PDP port.
- Use `null` for devices that do not consume a power distribution channel directly, such as
  CANcoders.

Bad entries here cause misleading power logs and can make CAN health debugging much harder.

## SensorConstants

`SensorConstants` holds configuration values for robot-wide sensors that do not naturally belong to
one mechanism:

- `kRioAccelerometerSampleRateHz`

Most teams can leave this alone. Increase it only if you have a specific acceleration or jerk
logging need and have verified that the extra sampling work is worth it.

## OperatorConstants

`OperatorConstants` configures driver feel and command-level control preferences. The physical
driver controller type is selected separately by `frc.robot.util.RBSIController`, which detects an
Xbox, PS4, or PS5 controller on the driver port at robot startup and exposes the same semantic
actions to `RobotContainer`.

- `kDriveStyle`
- `kJoystickDeadband`
- `kTurnSensitivity`
- `kJoystickSlewRateLimit`
- `kRobotRelativeNudgeSpeedMetersPerSec`
- `kAutopilotDemoXOffsetMeters`
- driver/operator switch IDs
- `MULTI_TOGGLE`

Tune this section with drivers:

- Use either an Xbox, PS4, or PS5 controller on driver port 0. RBSI maps Xbox `A/B/X/Y` to
  PlayStation `Cross/Circle/Square/Triangle`, and Xbox bumpers to PlayStation `L1/R1`.
- Set `kDriveStyle` to the preferred boot/default drive style. `TANK` uses the left stick for
  translation and the right stick for turning; `GAMER` uses the right stick for translation and the
  left stick for turning. RBSI also exposes this choice as the `Drive Style` dashboard chooser, so
  teams can switch between drivers without recompiling or redeploying.
- Adjust `kJoystickDeadband` until joystick drift disappears without making the robot feel numb.
- Adjust `kTurnSensitivity` and `kJoystickSlewRateLimit` after real driver practice.
- Adjust `kRobotRelativeNudgeSpeedMetersPerSec` to change the POV nudge speed from a single place.
- Adjust or remove `kAutopilotDemoXOffsetMeters` when replacing the example bumper drive-to-pose
  binding with a game-specific target.
- Verify all console switches against the actual HID device.

## ControllerButtonConstants

`ControllerButtonConstants` maps robot actions to controller-agnostic physical inputs from
`RBSIController`. Teams should usually remap controls here instead of editing `RobotContainer` or
`RBSIController`.

Physical button names are based on where the control sits on the gamepad:

| RBSI name | Xbox | PS4/5 |
| --- | --- | --- |
| `SOUTH_FACE` | `A` | `Cross` |
| `EAST_FACE` | `B` | `Circle` |
| `WEST_FACE` | `X` | `Square` |
| `NORTH_FACE` | `Y` | `Triangle` |
| `LEFT_BUMPER` | Left bumper | `L1` |
| `RIGHT_BUMPER` | Right bumper | `R1` |
| `LEFT_STICK` | Left stick press | `L3` |
| `RIGHT_STICK` | Right stick press | `R3` |
| `POV_LEFT` | D-pad left | D-pad left |
| `POV_RIGHT` | D-pad right | D-pad right |
| `POV_UP` | D-pad up | D-pad up |
| `POV_DOWN` | D-pad down | D-pad down |

Trigger axes are also named by position:

| RBSI axis | Xbox | PS4/5 |
| --- | --- | --- |
| `LEFT_TRIGGER` | Left trigger axis | `L2` axis |
| `RIGHT_TRIGGER` | Right trigger axis | `R2` axis |

Default robot-action mappings:

| Robot action constant | Default physical input |
| --- | --- |
| `BRAKE` | `SOUTH_FACE` |
| `ROBOT_RELATIVE` | `EAST_FACE` |
| `X_LOCK` | `WEST_FACE` |
| `ZERO_GYRO` | `NORTH_FACE` |
| `AUTOPILOT_DEMO` | `LEFT_BUMPER` |
| `RUN_FLYWHEEL` | `RIGHT_BUMPER` |
| `NUDGE_LEFT` | `POV_LEFT` |
| `NUDGE_RIGHT` | `POV_RIGHT` |
| `NUDGE_FORWARD` | `POV_UP` |
| `NUDGE_BACK` | `POV_DOWN` |

Operator controls use the same pattern. Add action-focused names to
`ControllerButtonConstants`, assign each action to a physical input, and bind those names to the
operator controller in `RobotContainer`. The action name should describe what the robot does, not
which controller button is pressed.

For example:

```java
public static final Button INTAKE_GAME_PIECE = Button.SOUTH_FACE;
public static final Button SCORE_GAME_PIECE = Button.EAST_FACE;
public static final Button REVERSE_INTAKE = Button.WEST_FACE;
public static final Button STOW_MECHANISM = Button.NORTH_FACE;
public static final Axis MANUAL_ARM_DOWN = Axis.LEFT_TRIGGER;
public static final Axis MANUAL_ARM_UP = Axis.RIGHT_TRIGGER;
```

Then bind those names in `RobotContainer`:

```java
operatorController.button(INTAKE_GAME_PIECE).whileTrue(intake.runIntakeCommand());
operatorController.button(SCORE_GAME_PIECE).whileTrue(superstructure.scoreCommand());
operatorController.axisTrigger(MANUAL_ARM_UP).whileTrue(arm.manualUpCommand());
```

This keeps `RBSIController` responsible only for translating physical controller layouts and keeps
team-specific control names in one readable constants section.

## DrivebaseConstants

`DrivebaseConstants` is one of the most important tuning sections. It controls physical drive
limits, drive PID constants, SysId settings, ramp rates, and odometry buffering.

High-priority robot-specific values:

- `kMaxLinearSpeedMetersPerSec`
- `kSlipCurrentAmps`
- `kWheelRadiusMeters`
- `kMaxLinearAccelMetersPerSecSq`
- `kStrafeP`, `kStrafeI`, `kStrafeD`
- `kSpinP`, `kSpinI`, `kSpinD`
- `kDriveP`, `kDriveD`, `kDriveV`, `kDriveA`, `kDriveS`, `kDriveT`
- `kSteerP`, `kSteerD`, `kSteerS`

Characterize or measure these:

- `kMaxLinearSpeedMetersPerSec`: use the real robot, not a theoretical free-speed estimate.
- `kWheelRadiusMeters`: use the wheel radius characterization routine.
- `kSlipCurrentAmps`: measure against carpet, ideally at an event carpet.
- drive feedforward gains: use drive characterization routines.

Odometry and disabled-vision behavior:

- `kPoseBufferHistorySecs`: history window used for latency compensation.
- `kDisabledVisionBlendAlpha`: how aggressively disabled vision pulls the pose estimate.
- `kDisabledVisionMaxJumpM` and `kDisabledVisionMaxJumpRad`: reject unreasonable disabled-vision
  jumps.
- `kDisabledCoastSeconds`: time to keep disabled coast behavior active.
- `kDisabledCoastMinSeconds`: minimum time before stationary detection can end the coast phase.
- `kDisabledVisionCoastBlendAlpha`: gentler disabled-vision blend while the robot is still
  coasting.
- stationary detection constants: tune if disabled pose behavior ends too quickly or too slowly.

Do not blindly copy drive gains between robots. Module type, wheel type, gearing, mass, current
limits, and CTRE license status all matter.

Drive characterization helpers also use constants here:

- `kSysIdPreRunStopSecs`: short stop/settle period before WPILib SysId drive tests.
- `kFeedforwardCharacterizationStartDelaySecs`: module-orientation delay before simple drive
  feedforward characterization begins collecting samples.
- `kFeedforwardCharacterizationRampRateVoltsPerSec`: voltage ramp rate for simple drive
  feedforward characterization.
- `kWheelRadiusCharacterizationStartDelaySecs`: module-orientation delay before wheel radius
  characterization begins measuring.
- `kWheelRadiusCharacterizationMaxVelocityRadPerSec`: maximum robot spin speed during wheel radius
  characterization.
- `kWheelRadiusCharacterizationRampRateRadPerSecSq`: spin-speed slew rate during wheel radius
  characterization.

## FlywheelConstants

`FlywheelConstants` configures the example flywheel subsystem:

- `kIdleMode`
- `kGearRatio`
- `kMaxVoltage`
- `kClosedLoopRampPeriodSecs`
- `kOpenLoopRampPeriodSecs`
- SysId settings
- CTRE Motion Magic Velocity settings
- real feedforward and feedback gains
- sim feedforward and feedback gains
- sim plant gearing and moment of inertia

Tune this section when using the example flywheel:

- `kGearRatio`: must match motor rotations to mechanism rotations.
- `kMaxVoltage`: voltage clamp for simulation and mechanism safety expectations.
- `kMotionMagicAccelerationRotPerSecSq` and `kMotionMagicJerkRotPerSecCubed`: CTRE Motion Magic
  Velocity profile settings for TalonFX control.
- `kRealS`, `kRealV`, `kRealA`: copy from WPILib SysId voltage characterization.
- `kRealP`, `kRealD`: tune after feedforward is correct.
- `kSimS`, `kSimV`, `kSimA`, `kSimP`, `kSimD`: tune separately for simulation.
- `kSimGearing` and `kSimMomentOfInertiaKgMetersSq`: tune when changing the example simulated
  flywheel's mass, radius, gearing, or motor.

See `doc/RBSI-SysId.md` for how to run the flywheel SysId routines and apply the results.

## AutoConstants

`AutoConstants` configures autonomous/pathing frameworks:

- `kPathPlannerConfig`
- `kChoreoDrivePID`
- `kChoreoSteerPID`
- `kAutopilot`

Tune this section after the drivetrain constants are correct. PathPlanner and Choreo depend on
accurate robot mass, moment of inertia, wheel radius, friction, current, and drive speed constants.

Pay special attention to:

- PathPlanner `RobotConfig`: depends on `RobotConstants`, `DrivebaseConstants`, and
  `SwerveConstants`.
- `kChoreoDrivePID` and `kChoreoSteerPID`: tune for trajectory tracking.
- Autopilot constraints and tolerances: tune for teleop drive-to-pose behavior.

## VisionConstants

`VisionConstants` configures AprilTag trust, filtering, and measurement uncertainty:

- `kTrustedTags`
- trusted/untrusted standard deviation scaling
- ambiguity and target logging thresholds
- field border and z-height margins
- linear/angular standard deviation baselines
- MegaTag2 standard deviation scaling

Tune these after cameras are mounted and calibrated:

- `kTrustedTags`: choose tags that are reliable for your game strategy and field side.
- `kMaxAmbiguity`: reject single-tag observations that are too ambiguous.
- `kMaxZErrorMeters`: reject pose estimates with physically unreasonable height.
- `kLinearStdDevBaseline` and `kAngularStdDevBaseline`: tune how much vision influences pose.
- trusted/untrusted scale factors: tune tag family confidence.

If the robot pose jumps, first verify camera transforms in `Cameras`, then revisit these noise and
gate constants.

## Cameras

`Cameras` defines PhotonVision camera configurations:

- camera name
- robot-to-camera transform
- per-camera standard deviation factor
- simulation camera properties

This section is critical for vision accuracy:

- `robotToCamera` must match the physical mounting location and angle.
- Rotation values are radians.
- Camera names must match the PhotonVision UI.
- Simulation calibration should roughly match the real camera.

Limelight users should configure camera transforms in the Limelight web UI where appropriate, but
the robot code still depends on `VisionConstants` for filtering and trust rules.

## DeployConstants

`DeployConstants` names deploy subdirectories:

- `kAprilTagDir`
- `kChoreoDir`
- `kPathPlannerDir`
- `kYagslDir`

Most teams should not change these unless they intentionally reorganize deploy files.

## Recommended Tuning Order For A New Robot

1. Select `robotType`, `swerveType`, `autoType`, `visionType`, and `phoenixPro`.
2. Configure `CANBuses` and `RobotDevices`.
3. Set `RobotConstants` mass, moment of inertia, wheel friction, and sensor orientations.
4. Set `SensorConstants` and `PowerConstants`.
5. Tune `OperatorConstants` with driver feedback.
6. Characterize and tune `DrivebaseConstants`.
7. Configure and validate cameras in `Cameras`.
8. Tune `VisionConstants`.
9. Tune `AutoConstants` after drive and vision are stable.
10. Tune mechanism sections such as `FlywheelConstants`.
11. Re-run tests and simulation after each major tuning pass.

## Common Failure Modes

- Wrong CAN bus name: devices appear disconnected or health logs look empty.
- Wrong power port: current logs blame the wrong subsystem.
- Wrong IMU orientation: heading or acceleration appears rotated.
- Wrong camera transform: vision updates pull pose in the wrong direction.
- Wrong gear ratio: velocity control and SysId constants do not transfer correctly.
- Untuned max speed or wheel radius: autonomous path following misses distance targets.
- Over-trusting vision: pose jumps toward bad tag solves.
- Under-trusting vision: odometry drift is never corrected.

## Related Pages

- [RBSI-Drive.md](RBSI-Drive.md): drivetrain constants, Phoenix Tuner X files,
  odometry, and characterization.
- [RBSI-Vision.md](RBSI-Vision.md): camera constants and pose-estimation tuning.
- [RBSI-Autonomous.md](RBSI-Autonomous.md): PathPlanner, Choreo, and Autopilot
  constants in match context.
