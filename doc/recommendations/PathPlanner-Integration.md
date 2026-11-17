# PathPlanner Integration Recommendations

This document recommends a practical architecture for using PathPlanner with WPILib's command-based framework. The goal is to keep routine autonomous paths easy to author and test while supporting configurable autos, runtime navigation, and clean driver takeover.

## Recommendation summary

Use PathPlanner as the common path format and trajectory-following backend. Keep robot behavior, mechanism sequencing, and high-level route selection in WPILib commands and project code.

The recommended flow is:

```text
Strategy / dashboard selection / vision state
                |
                v
  Project route selector or custom waypoint generator
                |
                v
PathPlannerPath + AutoBuilder follow/pathfind command
                |
                v
WPILib CommandScheduler + drivetrain requirements
                |
                v
        Drivetrain and mechanisms
```

This avoids making the PathPlanner GUI the only place autonomous behavior can be defined, while preserving its field editor, constraints, trajectory generation, telemetry, and alliance handling.

## 1. Keep authored paths as the shared route format

Store normal paths and autos in `src/main/deploy/pathplanner`. Use `AutoBuilder.followPath(...)` for the standard autonomous sequence.

For more flexible behavior, generate or select waypoints in code, construct a `PathPlannerPath`, and pass it to the same AutoBuilder follower. This permits strategy code to choose a destination or corridor at runtime without reimplementing drivetrain control.

Use this approach for:

- Selecting among scoring, pickup, or transit poses from field state.
- Applying a small number of known route modifiers, such as a safe passage around a field feature.
- Building configurable autos from tested path segments.

Do not generate a new path every scheduler cycle. Build a command once when the route decision changes, then schedule that command.

## 2. Separate fixed-path auto from dynamic navigation

Use fixed authored paths where timing and route shape matter, such as crossing narrow field regions, aligning to a scoring target, or coordinating a multi-step autonomous routine. Use `AutoBuilder.pathfindToPose(...)` only for an open-field movement to a staging pose or for driver-initiated alignment.

For a route that must pass through an obstacle-constrained corridor, break it into segments:

1. Pathfind to a safe staging pose before the corridor, if needed.
2. Follow an authored path through the corridor.
3. Continue with the next scoring or pickup segment.

This reduces the chance that a recovery maneuver shortcuts through an obstacle. A full dynamic replanning system should be a later experiment, not the first recovery mechanism.

## 3. Make WPILib requirements the source of command ownership

Every command that drives the robot should require the drivetrain. Driver-control commands should also require the drivetrain.

To allow immediate driver takeover, create a deadbanded `Trigger` for meaningful joystick motion and schedule the driver-control command from that trigger. WPILib's `CommandScheduler` will interrupt the active pathfinding or path-following command because both require the drivetrain.

Avoid global `cancelAll()` as the normal takeover mechanism. It can unintentionally stop mechanism commands and obscures which command owns the drivetrain.

## 4. Use NamedCommands for mechanisms, with normal command rules

Register mechanism actions with `NamedCommands.registerCommand(...)` before constructing the PathPlanner auto chooser. Each registered action should be an ordinary, well-tested WPILib command.

NamedCommands are appropriate for actions such as starting an intake, setting a shooter state, waiting for readiness, or running a score sequence. They are not callbacks that bypass subsystem requirements.

When building a parallel PathPlanner group:

- A path-following command owns the drivetrain.
- A mechanism action may run in parallel only when it owns different subsystems.
- Two commands that both require the same subsystem must run sequentially or be combined into one command.

Test every auto after adding or modifying a NamedCommand. Requirement conflicts can prevent the auto from loading or cancel it at runtime.

## 5. Initialize and correct pose deliberately

Configure AutoBuilder with the project pose estimator's pose supplier and reset-pose method. When the PathPlanner auto is configured to reset odometry, that callback should establish the expected starting pose through the estimator.

Vision should then continue to correct the estimator during auto; it should not be the only way the robot learns its initial pose. Before relying on vision corrections:

- Reject implausible or ambiguous measurements.
- Prefer fewer high-quality observations over frequent poor ones.
- Verify that the exact pose supplier passed to AutoBuilder includes accepted vision measurements.
- Log planned pose, estimated pose, and drive command together for review.

## 6. Add configurable autos after fixed autos are reliable

A small dashboard or NetworkTables tool can assemble pre-tested path segments into a match-specific routine without a redeploy. Limit its freedom deliberately:

- Offer named, validated segments instead of arbitrary poses.
- Validate that the selected segment end and next segment start are compatible.
- Show the resulting sequence, estimated duration, and any mechanism events before enable.
- Make the robot use a safe default auto if the configuration is incomplete or invalid.

Treat PathPlanner hot reload as a development convenience. Keep the competition configuration mechanism explicit, reviewable, and disabled-safe.

## 7. Validate multi-robot autonomous compatibility offline

Before an event, export or inspect the complete PathPlanner directories for likely alliance partners. Overlay the autos with their timing and robot footprints to find start-zone or transit collisions.

This should be an offline strategy tool, not a runtime collision-avoidance system. Record safe pairings as a simple alliance-autonomous checklist.

## Implementation plan

### Phase 1: Baseline integration

1. Configure `AutoBuilder` with the pose supplier, reset-pose callback, robot-relative chassis-speed supplier, robot-relative drive consumer, and drivetrain requirement.
2. Add a PathPlanner auto chooser and one simple authored path.
3. Verify blue/red behavior, reset-pose behavior, and a straight-line path at conservative constraints.
4. Log planned trajectory, estimated pose, and commanded chassis speeds.

### Phase 2: Command composition

1. Register one simple NamedCommand for a mechanism state change.
2. Add it sequentially to an auto, then add a different-subsystem action in parallel with a path.
3. Add joystick-triggered driver takeover with drivetrain requirements.
4. Test interruptions in simulation and on the robot.

### Phase 3: Dynamic routing

1. Define named staging poses and obstacle-safe corridors.
2. Add a single `pathfindToPose` command for teleop alignment or an open-field transition.
3. Chain the command into an authored corridor path.
4. Measure path-generation latency and behavior after a simulated pose disturbance.

### Phase 4: Configurable autos

1. Build a small NetworkTables interface that selects from approved segments.
2. Validate selections before autonomous enable.
3. Keep a static PathPlanner auto as the fallback.
4. Trial the workflow in practice matches before using it at competition.

## Verification checklist

- `AutoBuilder` consumes **robot-relative** chassis speeds and outputs robot-relative drive commands.
- The pose supplier is the same estimator used by vision fusion.
- An auto can reset to its known starting pose with vision unavailable.
- Every NamedCommand has correct subsystem requirements.
- Driver input interrupts only drivetrain automation, not unrelated mechanism commands.
- Dynamic pathfinding is created once per request, never continuously from `whileTrue`.
- Fixed corridor paths have conservative constraints and are tested after collision or pose-error disturbances.
- All intended alliance auto combinations are reviewed for time-and-space conflicts.

## Further reading

- [Chief Delphi: custom waypoint generation fed into `AutoBuilder.followPath`](https://www.chiefdelphi.com/t/4188s-take-on-autonomous-pathfinding/515274)
- [Chief Delphi: pose-parameterized and time-parameterized interpretations of PathPlanner paths](https://www.chiefdelphi.com/t/autonomous-development/518886)
- [Chief Delphi: configurable NetworkTables-based autos](https://www.chiefdelphi.com/t/frc-5000-hammerheads-2026-build-thread-open-alliance/507502?page=16)
- [Chief Delphi: multi-auto collision visualization](https://www.chiefdelphi.com/t/introducing-multipathplanner-visualizer/519383)
- [Chief Delphi: WPILib requirements for pathfinding interruption](https://www.chiefdelphi.com/t/pathfinding-issues/515746)
- [Chief Delphi: NamedCommand requirement behavior](https://www.chiefdelphi.com/t/certain-namedcommands-disabling-auto-for-an-unknown-reason/520111)
- [Chief Delphi: AutoBuilder pose reset with a vision estimator](https://www.chiefdelphi.com/t/pathplanner-initial-pose-question/515516)
