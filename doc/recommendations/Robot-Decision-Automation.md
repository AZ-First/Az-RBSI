# Robot Decision Automation Recommendations

This document recommends how Az-RBSI can automate mechanism coordination and local gameplay decisions while preserving clear driver authority. The recommendations are based on automation systems shared by teams during the 2026 REBUILT season.

## Recommendation summary

Automate execution and bounded local decisions rather than attempting to make the robot independently play the whole match. The driver should request a high-level intent such as acquiring fuel, scoring, passing, traversing, or climbing. The robot should select the appropriate target, mechanism states, setpoints, safety actions, and completion conditions.

```text
Driver intent + match state + field observations
                       |
                       v
              World-state model
                       |
                       v
             Intent/decision policy
                       |
                       v
          Superstructure coordinator
                       |
                       v
        Mechanism state machines and IO
```

The recommended implementation order is:

1. Match and hub-state awareness.
2. One-button scoring and intake intents.
3. Automatic hopper and shooter-tower filling.
4. Automatic stow, shot-inhibit, and safety interlocks.
5. Trench, shooting, and climbing assists.
6. Jam and unbeach recovery reflexes.
7. Perception-based fuel-cluster selection.

## 1. Build a shared world-state model

Create a single read-only model containing the information needed to make decisions. It should be updated every robot loop from authoritative subsystem state, field state, and accepted sensor observations.

Useful inputs include:

- Estimated field pose, chassis velocity, pitch, and roll.
- Current match phase, alliance, hub state, and time until the next shift.
- Estimated projectile time of flight from the current position.
- Hopper state, shooter-tower occupancy, and mechanism readiness.
- Camera, sensor, and localization confidence or health.
- Field-zone membership and proximity to the trench, bump, tower, or field boundary.
- Battery voltage, total current, and available power profile.
- Active driver intent and any manual overrides.

Do not let decision code query hardware devices directly. Subsystems should validate and publish their state; the decision layer should consume a coherent snapshot.

## 2. Use high-level driver intents

Controller bindings should communicate what the driver wants to accomplish, not every actuator step required to accomplish it.

Recommended intents are:

```text
IDLE
ACQUIRE_FUEL
SCORE_HUB
PASS_FUEL
TRAVERSE_TRENCH
CLIMB
SAFE_DRIVE
RECOVER
```

For example, a score request should allow the robot to:

1. Determine whether the hub is active or will become active soon.
2. Select a field-relative shot or preset fallback.
3. Aim with the turret or drivetrain.
4. Set hood and flywheel goals.
5. Wait until the shot solution and mechanisms are ready.
6. Feed fuel only while launch conditions remain valid.

The operator should retain explicit overrides for target selection, mechanism recovery, sensor failure, and disabling individual automation features.

## 3. Coordinate mechanisms through a superstructure

Use a superstructure coordinator above the individual subsystems. It should translate the selected intent into well-defined mechanism and chassis goals.

Teams 11010 and 2910 reported that state-machine or superstructure designs made automatic align-and-score actions easier to build, understand, and debug than collections of stateful commands and boolean flags. Team 11010 separated action, primary-mechanism, and chassis states, while Team 2910 emphasized that state machines keep behavior predictable as sequences become more sophisticated. [Team 11010 software architecture](https://www.chiefdelphi.com/t/team-bobcats-11010-2026-season-build-blog/508880) [Team 2910 code-release discussion](https://www.chiefdelphi.com/t/team-2910-code-release-2026/521778)

The superstructure should own coordination policy, but individual subsystems should continue to own control loops, measured state, limits, and fault reporting.

Every transition should define:

- Preconditions.
- Requested subsystem goals.
- Completion conditions.
- Timeout behavior.
- Interruption behavior.
- A safe fallback.

## 4. Make the robot aware of match timing

Track match time locally and combine it with alliance and FMS game data to determine the current shift, hub state, and time until the next transition. Provide a manual correction for missing or incorrect game data.

Mechanical Advantage 6328 adjusted the usable launch window using projectile time of flight, hub processing time, and the scoring grace period. This let the robot launch before official activation so fuel arrived when the hub began counting. Their system also alerted the drive team and accepted manual overrides when FMS game data was unavailable. [6328's match-aware scoring system](https://www.chiefdelphi.com/t/frc-6328-mechanical-advantage-2026-build-thread/509595?page=28)

Team 4744 used match timing to switch automatically between scoring and delivery modes before hub-state changes. Their delivery logic also stopped shooting when field geometry made the target unreachable. [Team 4744 hub-aware behavior](https://www.chiefdelphi.com/t/ninjas-4744-2026-build-thread-open-alliance/505741?page=4)

Az-RBSI should expose derived state rather than repeating timing logic throughout commands:

```text
currentShift
hubActive
timeUntilHubChange
launchWindowOpen
prepareToScore
prepareToPass
```

## 5. Automate filling, indexing, and compaction

The robot does not need an exact fuel count to make useful internal decisions. Start with operational states inferred from beam breaks, range sensors, roller velocity, and motor current:

```text
EMPTY
ACCEPTING
TOWER_FULL
HOPPER_FULL_OR_STALLED
FEEDING
JAM_SUSPECTED
```

Team 581 combined a CANrange sensor with a retroreflective shooter-tower sensor. The robot automatically ran rollers until the tower was filled, increasing usable storage without ejecting fuel prematurely. [Team 581 code release](https://www.chiefdelphi.com/t/581-blazing-bulldogs-2026-cad-and-code-release/521762)

Recommended behavior:

- Run collection and floor rollers while accepting fuel.
- Stop or reduce compaction when the tower and hopper are full.
- Stage fuel before a scoring window.
- Detect a likely jam from commanded speed, measured velocity, and current.
- Reverse or pulse only the affected stage for a bounded time.
- Escalate to driver recovery after repeated failures.

## 6. Add reflexive safety and recovery

Implement reflexes as guarded overrides below the strategic decision layer. A reflex should temporarily change execution while preserving the original driver intent when recovery is possible.

Recommended reflexes include:

```text
hood unsafe near trench       -> stow hood
shot solution invalid         -> inhibit feeder
localization confidence low   -> use preset shot or manual drive
pitch/roll exceeds threshold  -> enter unbeach recovery
mechanism fails to progress   -> stop, retry once, then alert
battery voltage margin low    -> select constrained power profile
```

Team 581 used its Pigeon IMU to detect when the robot became beached, briefly drove away from the obstruction, and then resumed its autonomous routine. The same team fell back to preset shots when cameras failed and provided dashboard controls to disable complex functionality. [Team 581 recovery and fallback systems](https://www.chiefdelphi.com/t/581-blazing-bulldogs-2026-cad-and-code-release/521762)

Team 4096 automatically lowered its shooter hood near the trench and experimented with pointing the intake along the robot's velocity vector. [Team 4096 automation](https://www.chiefdelphi.com/t/frc-team-4096-ctrl-z-2026-build-thread-open-alliance/512198)

All automatic recovery must have a timeout, retry limit, driver-visible reason, and immediate manual override.

## 7. Make controls context-sensitive but explainable

A high-level request may resolve differently depending on field position and robot capability. One 2026 control-scheme proposal used the same trigger to score in the alliance zone or pass from elsewhere; if the turret was unavailable, the drivetrain performed the aiming instead. [REBUILT control-scheme discussion](https://www.chiefdelphi.com/t/driver-opperator-control-schemes-for-rebuilt/514681)

Represent the decision as a result containing its reasoning:

```java
public record Decision<T>(T value, String reason, double confidence) {}
```

Log the request, selected action, reason, confidence, rejected alternatives, and any override. The dashboard should show concise statements such as:

```text
PASS_FUEL: hub inactive for 14.2 s
PRESET_SHOT: vision confidence below threshold
FEED_INHIBITED: flywheel not ready
RECOVERING: pitch exceeded beach threshold
```

## 8. Use driver assists for constrained motion

The most useful drive assists preserve driver translation while automating a constrained dimension:

- Aim the robot or turret at the hub while the driver translates.
- Center laterally in the trench while preserving forward/backward control.
- Snap to a safe heading for bump traversal.
- Align the climber while preserving an escape input.
- Point an intake toward the motion vector when that measurably improves collection.

Team 4744 automatically enabled trench centering based on field position, without requiring a button. Team 581 also developed trench, wall-intake, bump-crossing, and intake-orientation assists. [Team 4744 trench automation](https://www.chiefdelphi.com/t/ninjas-4744-2026-build-thread-open-alliance/505741?page=4) [Team 581 driver-assist results](https://www.chiefdelphi.com/t/581-blazing-bulldogs-2026-cad-and-code-release/521762)

Each assist should blend or constrain driver input rather than unexpectedly taking full control. Entry and exit conditions need hysteresis so the assist does not chatter near a field-zone boundary.

## 9. Select fuel targets with bounded autonomy

Team 581 detected fuel clusters using a Limelight-hosted Python pipeline, converted observations into field poses, aggregated them into a cluster map, and selected among three known autonomous lanes. The team evaluated clusters using estimated balls collected per second of driving, which filtered out low-value targets. [Team 581 cluster-map system](https://www.chiefdelphi.com/t/581-blazing-bulldogs-2026-cad-and-code-release/521762)

Az-RBSI should follow the same bounded approach. Perception should rank known-safe tasks rather than produce unconstrained robot behavior.

A starting utility function is:

```text
utility =
    expectedFuel / estimatedCompletionTime
    - collisionRisk
    - localizationUncertainty
    - routeSwitchingPenalty
```

Require a minimum confidence and improvement margin before changing targets. Once committed to a short acquisition maneuver, finish it unless a safety condition or driver override occurs.

## 10. Manage electrical power as a robot resource

Team 581 dynamically assigned current limits based on the active action, prioritizing shooter consistency during scoring and feeder throughput while passing. [Team 581 Power Manager](https://www.chiefdelphi.com/t/581-blazing-bulldogs-2026-cad-and-code-release/521762)

Az-RBSI should begin with discrete, tested power profiles:

| Profile | Priority |
| --- | --- |
| `NORMAL` | Balanced driving and mechanism operation. |
| `SCORING` | Shooter and feeder consistency. |
| `TRAVERSAL` | Drivetrain control and mechanism stow. |
| `CLIMBING` | Climber and stabilization. |
| `LOW_VOLTAGE` | Preventing brownout and preserving control. |

Select profiles from superstructure state, battery margin, and expected current demand. Begin with conservative fixed limits and logging before attempting frequent runtime reconfiguration.

## 11. Encode coordination mechanically when appropriate

If two mechanical actions should nearly always happen together, consider linking them mechanically before adding another actuator and control loop.

Team 157 mechanically coupled intake deployment to hopper-wall extension, translating one rotational input into coordinated linear movement. Other 2026 designs used intake retraction to compact fuel toward the shooter. [Team 157 coupled intake and hopper](https://www.chiefdelphi.com/t/frc-157-the-aztechs-2026-build-thread-open-alliance/509425/25) [2026 intake and hopper discussion](https://www.chiefdelphi.com/t/frc-8575-offseason-cad-practice/520816)

Mechanical coupling can remove sensors, motors, commands, and failure modes. It should still be reviewed for impact tolerance, binding, maintenance access, and whether independent motion is needed for recovery.

## Suggested Az-RBSI components

```text
FieldState
  MatchPhase, hub state, transition timing, field zones

RobotWorldModel
  Pose, velocity, mechanism state, fuel state, health, power

RobotIntent
  Acquire, score, pass, traverse, climb, safe, recover

DecisionPolicy
  Resolves intent into a target and action with reason/confidence

SuperstructureCoordinator
  Executes action states and coordinates subsystem goals

AutomationReflexes
  Safety interlocks, jam recovery, unbeach, sensor fallback

AutomationOverrides
  Per-feature disable, preset fallback, full manual recovery
```

Keep the world model and decision policy free of command-framework types so they can be used by both Commands V2 and V3 infrastructure.

## Implementation phases

### Phase 1: Deterministic automation

1. Add match/hub-state tracking with manual correction.
2. Define high-level driver intents.
3. Implement a superstructure for intake, scoring, passing, and safe drive.
4. Add automatic staging and basic mechanism interlocks.
5. Log every state transition and rejected action.

### Phase 2: Reflexes and assists

1. Add hood/intake stow zones.
2. Add jam detection with bounded recovery.
3. Add trench and climb alignment assists.
4. Add preset fallbacks for failed vision or localization.
5. Measure whether each assist improves success rate or cycle time.

### Phase 3: Predictive behavior

1. Add projectile time-of-flight to scoring-window calculations.
2. Add iterative shoot-on-the-move compensation.
3. Add action-based power profiles.
4. Add bounded fuel-cluster ranking from field observations.

## Verification and acceptance criteria

Every automation feature should demonstrate:

- A measurable improvement in cycle time, success rate, or driver workload.
- Deterministic behavior for identical inputs.
- Hysteresis around thresholds and field-zone boundaries.
- A timeout and retry limit for recovery actions.
- A visible reason for its current decision.
- Safe behavior when a required sensor becomes invalid.
- An immediate driver or operator override.
- Simulation coverage and logged practice-match evidence.

Team 581 deprecated snake mode, wall-intake assist, and bump-crossing assist after practice showed that trained drivers performed as well or better, or that the automation hurt performance under defense. This is a useful standard for Az-RBSI: automation should remain only when evidence shows that it improves the robot in realistic play. [Team 581 deprecated automation findings](https://www.chiefdelphi.com/t/581-blazing-bulldogs-2026-cad-and-code-release/521762)

## References

- [Mechanical Advantage 6328: match-aware scoring windows](https://www.chiefdelphi.com/t/frc-6328-mechanical-advantage-2026-build-thread/509595?page=28)
- [Team 581: cluster mapping, filling, power management, recovery, and fallbacks](https://www.chiefdelphi.com/t/581-blazing-bulldogs-2026-cad-and-code-release/521762)
- [Team 11010: superstructure, simulation, shot adjustment, and climb alignment](https://www.chiefdelphi.com/t/team-bobcats-11010-2026-season-build-blog/508880)
- [Team 2910: state-machine coordination](https://www.chiefdelphi.com/t/team-2910-code-release-2026/521778)
- [Team 4096: mechanism protection and shoot-on-the-move groundwork](https://www.chiefdelphi.com/t/frc-team-4096-ctrl-z-2026-build-thread-open-alliance/512198)
- [Team 4744: match-aware behavior and trench automation](https://www.chiefdelphi.com/t/ninjas-4744-2026-build-thread-open-alliance/505741?page=4)
- [2026 driver-control automation discussion](https://www.chiefdelphi.com/t/driver-opperator-control-schemes-for-rebuilt/514681)
- [Team 157: mechanically coupled intake and hopper](https://www.chiefdelphi.com/t/frc-157-the-aztechs-2026-build-thread-open-alliance/509425/25)
