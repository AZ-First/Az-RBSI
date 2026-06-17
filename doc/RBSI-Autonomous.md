# Az-RBSI Autonomous Guide

This page describes the autonomous options included in RBSI and how to choose
between Manual, PathPlanner, Choreo, and Autopilot workflows.

Select the active autonomous stack in `Constants.java`:

```java
private static AutoType autoType = AutoType.MANUAL;
```

Supported values:

- `MANUAL`: teams write their own command sequence.
- `PATHPLANNER`: use PathPlanner and PathPlannerLib.
- `CHOREO`: use Choreo trajectories.

Autopilot is used as a teleop drive-to-pose helper and can also inspire custom
autonomous commands.

## Match Execution Flow

RBSI follows the normal WPILib command-based lifecycle:

1. `Robot` starts AdvantageKit logging and constructs `RobotContainer`.
2. `RobotContainer` constructs subsystems, configures PathPlanner or Choreo for
   the selected `AutoType`, registers named commands, builds dashboard choosers,
   and binds driver controls.
3. Every `robotPeriodic()` loop runs virtual subsystems first, then
   `CommandScheduler.getInstance().run()`. This lets IMU and odometry samples
   reach pose estimation before commands and vision consumers need them.
4. `autonomousInit()` cancels any commands left from disabled/practice, sets
   drive brake mode, resets heading-controller state, opens the vision pose gate,
   reads the selected autonomous command, and schedules it.
5. `teleopInit()` cancels the stored autonomous command, restores brake mode,
   resets heading-controller state, and lets the default drive command take over.

This means autonomous commands should own only their intended subsystems. A
command that requires `Drive` will interrupt the default drive command while it
runs, and teleop will cancel the stored auto before the driver command resumes.

## Manual Autos

`MANUAL` is the simplest mode. RBSI does not construct a PathPlanner chooser or
Choreo factory. Use this when:

- the robot is in early bring-up,
- you are debugging drivetrain behavior,
- your team wants simple command-based autos,
- you want to avoid pathing dependencies until drive and vision are stable.

The example `simpleAuto()` in `RobotContainer` is a placeholder for teams that
want to build autos directly with commands.

## PathPlanner

When `autoType` is `PATHPLANNER`, RBSI configures PathPlanner `AutoBuilder` in
`Drive` and publishes a logged dashboard chooser:

```java
new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser())
```

PathPlanner depends on:

- accurate robot pose,
- correct module translations,
- correct wheel radius,
- correct max speed,
- correct PathPlanner `RobotConfig`,
- alliance-aware path flipping.

Important constants:

- `AutoConstants.kPathPlannerConfig`
- `DrivebaseConstants.kMaxLinearSpeedMetersPerSec`
- `DrivebaseConstants.kWheelRadiusMeters`
- `DrivebaseConstants.kSlipCurrentAmps`
- `RobotConstants.kMass`
- `RobotConstants.kMomentOfInertiaKgMetersSq`
- `RobotConstants.kWheelCoefficientOfFriction`

PathPlanner chooser entries also include drive and flywheel SysId routines.
This is intentional: characterization commands are easiest to run from the
same dashboard path used for autonomous selection.

RBSI configures PathPlanner `AutoBuilder` in `Drive` with:

- `this::getPose`
- `this::resetPose`
- `this::getChassisSpeeds`
- `(speeds, feedforwards) -> runVelocity(speeds)`
- `PPHolonomicDriveController`
- `AutoConstants.kPathPlannerConfig`
- alliance-aware path flipping
- the `Drive` subsystem requirement

PathPlannerLib's holonomic AutoBuilder expects robot-relative measured speeds
and robot-relative output speeds. `Drive.getChassisSpeeds()` returns
robot-relative speeds from the module states, and `Drive.runVelocity(...)`
accepts robot-relative chassis speeds, so the callback pair is intentionally
not field-relative.

## Choreo

When `autoType` is `CHOREO`, RBSI constructs an `AutoFactory` and uses the
Choreo sample-following support in `Drive`.

Important constants:

- `AutoConstants.kChoreoDrivePID`
- `AutoConstants.kChoreoSteerPID`

Choreo should be tuned after the drivetrain already tracks pose accurately.
If a Choreo path misses badly, do not start by changing PID constants. First
verify odometry, wheel radius, gyro orientation, and pose reset behavior.

RBSI constructs the Choreo `AutoFactory` in `RobotContainer` with:

- `m_drivebase::getPose`
- `m_drivebase::resetPose`
- `m_drivebase::followTrajectory`
- alliance flipping enabled
- `m_drivebase` as the subsystem requirement

Each routine should reset odometry once at the start of the first trajectory.
The example routine does that with:

```java
routine.active().onTrue(Commands.sequence(pickupTraj.resetOdometry(), pickupTraj.cmd()));
```

Do not also reset pose in a separate command at the same trigger unless you
intend to override the path's starting pose.

## Autopilot

Autopilot is configured in `AutoConstants`:

- `kAPConstraints`
- `kAPProfile`
- `kAutopilot`

RBSI exposes several `AutopilotCommands.runAutopilot(...)` overloads for
drive-to-pose behavior. It logs useful values under `Autopilot/*`, including:

- current pose,
- final pose,
- robot speeds,
- output velocities,
- target angle,
- at-target state.

Tune Autopilot after driver controls and pose estimation are stable.

## Named Commands

PathPlanner named commands are registered before autos and paths are created.
Keep that ordering. If a path references a named command that is not registered
before the chooser is built, the path can fail to load or run incorrectly.

When adding mechanisms:

1. Add subsystem code.
2. Add commands.
3. Register named commands in `RobotContainer`.
4. Reference those exact names in PathPlanner.
5. Test each named command by itself before embedding it in a full auto.

## Pose Reset Rules

Autonomous pathing depends on a clean starting pose. RBSI avoids fighting
PathPlanner’s `AutoBuilder` pose handling by keeping starting-pose reset logic
centralized.

General rules:

- Let PathPlanner reset pose for PathPlanner autos.
- Let Choreo reset pose for Choreo autos.
- For manual autos, explicitly reset pose only when the command owns the
  starting condition.
- Avoid resetting pose from multiple commands at the same time.
- Use `Drive.resetPose(...)` for autonomous resets so the pose reset epoch and
  vision pose gate stay aligned.

Unexpected pose resets are one of the fastest ways to make a correct path look
wrong.

## Running Characterization Commands

When using PathPlanner mode, the auto chooser includes:

- drive wheel radius characterization,
- drive feedforward characterization,
- drive SysId routines,
- flywheel SysId routines.

Run these with the robot in a safe area and the Driver Station ready to disable.
Characterization commands intentionally command motors in ways that do not feel
like normal teleop driving.

## Recommended Autonomous Bring-Up Order

1. Start in `MANUAL` mode.
2. Verify robot-relative drive.
3. Verify field-relative drive.
4. Verify odometry on a straight-line push test.
5. Verify vision is not corrupting pose.
6. Characterize wheel radius and feedforward.
7. Switch to `PATHPLANNER` or `CHOREO`.
8. Run a short, slow straight path.
9. Run a slow turn path.
10. Add mechanism commands.
11. Increase speed only after repeatability is good.

## Troubleshooting

Auto does nothing:

- Check `autoType`.
- Check the correct chooser is visible.
- Check the selected auto is not `Commands.none()`.
- Check named commands are registered.

Path starts from the wrong place:

- Check starting pose reset.
- Check alliance flipping.
- Check field layout.
- Check odometry before auto starts.

Path tracks poorly:

- Verify wheel radius.
- Verify max speed.
- Verify gyro orientation.
- Tune drive PID only after the physical constants are right.

Robot follows path in mirror image:

- Check alliance-aware flipping logic.
- Check PathPlanner field coordinate assumptions.
- Check robot heading at auto start.

## Related Pages

- [RBSI-Drive.md](RBSI-Drive.md): odometry, robot-relative speed conventions,
  and drive characterization.
- [RBSI-Vision.md](RBSI-Vision.md): vision filtering before autonomous pose
  fusion.
- [RBSI-Constants.md](RBSI-Constants.md): `AutoConstants` and drive constants
  used by autonomous frameworks.
