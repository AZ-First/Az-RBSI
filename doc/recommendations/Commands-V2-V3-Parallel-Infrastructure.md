# Commands V2 and V3 Parallel Infrastructure Recommendations

This document recommends how Az-RBSI can support both WPILib Commands V2 and Commands V3 during the 2027 transition. It is intended to let established teams retain tested V2 code while allowing new teams to adopt V3 directly.

## Recommendation summary

Support both command frameworks in the repository, but do **not** let both frameworks command the same hardware in one deployed runtime profile.

Build framework-neutral IO and state services below two thin command-framework facades:

```text
                     Robot hardware and vendor APIs
                                  |
                                  v
                 IO classes + state/control services
                    (no Command or Mechanism imports)
                         /                    \
                        v                      v
        V2 subsystem facade               V3 mechanism facade
  org.wpilib.command2.*                 org.wpilib.command3.*
        |                                      |
        v                                      v
    Legacy V2 commands                    Native V3 commands
```

At robot startup, select **one framework owner per mechanism**. The default should be a complete V2 profile until V3 support has a full season of proven use. A V3 profile should be available for teams deliberately choosing V3, but it must own its drivetrain and mechanisms exclusively.

## Why separate facades are necessary

Commands V2 and V3 are separate frameworks. V2 uses `Subsystem` and `CommandScheduler`; V3 uses `Mechanism` and `Scheduler`, and V3 commands are coroutine-based. V3 must be run by calling `Scheduler.getDefault().run()` every robot loop and has strict single-thread requirements. [WPILib V3 Scheduler API](https://github.wpilib.org/allwpilib/docs/2027/java/org/wpilib/command3/Scheduler.html)

Both frameworks provide exclusive ownership of their required resources, but neither scheduler knows about the other scheduler's resource claims. A V2 drivetrain command and a V3 drivetrain command could therefore both issue motor outputs unless Az-RBSI prevents that configuration.

Do not solve this with command adapters that schedule V2 commands from V3 or vice versa. Such adapters obscure lifecycle, cancellation, priorities, requirements, and telemetry. Share **operations and state**, not command instances.

## 1. Establish framework-neutral robot layers

Move reusable behavior below the command framework boundary. The following code must not import `org.wpilib.command2.*` or `org.wpilib.command3.*`:

- `*IO` interfaces and real/sim/replay implementations.
- Sensor input snapshots and actuator output requests.
- Estimation, interpolation, kinematics, safety limits, and mechanism state machines.
- Pure calculations such as aiming, shot solutions, drive setpoint generation, and path/route selection.
- Logging and tunable-constant access.

The shared layer should expose explicit operations, for example:

```java
public interface FlywheelControl {
  void requestVelocity(AngularVelocity velocity);
  void stop();
  boolean atSetpoint();
}
```

The interface returns state and accepts intent; it does not return a V2 `Command` or V3 `Command`.

## 2. Provide a facade for each framework

For each command-controlled mechanism, provide one small facade in each framework package:

```text
frc.robot.control.shared/   Framework-neutral control and state services
frc.robot.control.v2/       SubsystemBase wrappers and V2 command factories
frc.robot.control.v3/       Mechanism wrappers and V3 command factories
frc.robot.bindings.v2/      V2 Trigger/controller bindings
frc.robot.bindings.v3/      V3 Trigger/controller bindings
```

The V2 facade may extend the existing `RBSISubsystem` or WPILib `SubsystemBase`. The V3 facade should extend `org.wpilib.command3.Mechanism` and build commands with `Mechanism.run(...)` or the staged-command builder. WPILib recommends mechanism factory methods for simple single-mechanism V3 commands. [WPILib V3 Command API](https://github.wpilib.org/allwpilib/docs/2027/java/org/wpilib/command3/Command.html)

Both facades delegate to the same framework-neutral control service. They must not be constructed together for a mechanism that can apply outputs.

## 3. Select a complete runtime profile

Use an explicit startup setting, such as `CommandFramework.V2` and `CommandFramework.V3`, in robot configuration. This is a deployment/build decision, not a dashboard toggle.

| Profile | Constructed facades | Scheduler loop | Intended users |
| --- | --- | --- | --- |
| `V2` (default) | V2 facades only | `CommandScheduler.getInstance().run()` | Teams carrying legacy code or using V2-only vendor integrations. |
| `V3` (opt-in) | V3 facades only | `Scheduler.getDefault().run()` | Teams deliberately adopting V3 for all command ownership. |
| Test harness | One framework per test fixture | Its corresponding isolated scheduler | Unit tests and migration verification. |

Do not offer a `MIXED` profile that constructs both facades for a drivetrain, arm, shooter, or any other output-owning mechanism. A read-only diagnostic service may be shared freely; motor control may not.

The robot lifecycle should call only the scheduler belonging to the selected profile. This makes command ownership visible in code review and eliminates accidental double scheduling.

## 4. Preserve the current V2 experience

Keep the existing V2 package names, bindings, command factories, and `RobotContainer` entry points stable. V2 should remain the documented default until the project has complete V3 examples, simulation coverage, vendor-library compatibility, and on-robot validation.

Migration must be additive:

1. Extract framework-neutral control logic without changing V2 public behavior.
2. Point existing V2 subsystems/commands at the extracted service.
3. Add a V3 facade and equivalent V3 example in a separate package.
4. Compare the two implementations in simulation and mechanism tests.
5. Permit V3 as an opt-in profile only after the comparison passes.

Do not require legacy teams to convert command groups, triggers, or autonomous code merely to obtain routine Az-RBSI updates.

## 5. Design native V3 examples, not V2-shaped V3 wrappers

V3 supports composition, priority, coroutine control flow, and declarative state machines. New V3 examples should teach those concepts directly rather than recreating V2 inheritance patterns.

For example, a V3 mechanism should expose focused intent commands such as `intake()`, `hold()`, `moveTo(...)`, or `idle()`. Multi-mechanism actions should be assembled from those commands, with requirements and priority visible at the composition site.

V3 command code must yield in periodic loops. V3's documentation warns that failing to yield can produce an unrecoverable loop, and that its scheduler must remain single-threaded. [WPILib V3 Command API](https://github.wpilib.org/allwpilib/docs/2027/java/org/wpilib/command3/Command.html)

## 6. Isolate integrations by framework compatibility

Treat PathPlanner, Choreo, AdvantageKit, vendor libraries, dashboards, and any custom command library as compatibility gates. Record the tested version and framework for each integration in a small compatibility matrix.

| Integration | V2 status | V3 status | Required evidence before enabling |
| --- | --- | --- | --- |
| Az-RBSI drivetrain commands | Supported baseline | Prototype first | Simulation, characterization, and real-robot drive tests. |
| Autonomous/path library | Existing V2 integration | Verify independently | A simple auto, alliance behavior, cancellation, and logging. |
| Logging/replay | Existing baseline | Verify independently | Replay and scheduler telemetry test. |
| Vendor motor library | Existing baseline | Usually framework-neutral | Real/sim IO test through the selected facade. |

Do not assume a library that returns a V2 `Command` can be used from V3. Keep external commands at the edge of the selected profile; expose any common calculations or data through the framework-neutral layer.

## 7. Build a migration test matrix

Every mechanism moved to the shared layer should pass the same behavioral tests through both facades:

- Unit test the framework-neutral control service without a command scheduler.
- Run V2 command tests with a V2 scheduler fixture.
- Run V3 command tests with an independent V3 `Scheduler` fixture.
- Simulate command interruption, disable/enable transitions, timeouts, and default behavior.
- Verify that exactly one facade can emit actuator outputs in each profile.
- Log and compare setpoint, measured state, and motor output traces for equivalent V2 and V3 scenarios.

Use direct, bounded tests for V3 coroutines. Avoid running V3 commands in worker threads or virtual threads; the scheduler documentation explicitly prohibits multithreaded use.

## Implementation plan

### Phase 0: Stabilize the boundary

1. Add `CommandFramework` configuration with `V2` as the only enabled production value.
2. Document the prohibition on dual command ownership.
3. Identify commands that contain reusable control logic and extract that logic into framework-neutral services.

### Phase 1: Pilot one low-risk mechanism

1. Choose a simple example mechanism, such as the example flywheel.
2. Keep its V2 subsystem and commands operating unchanged.
3. Add a V3 `Mechanism` facade that delegates to the same flywheel control service.
4. Add V2 and V3 simulation tests that verify the same speed request, stop behavior, and readiness signal.

### Phase 2: Add V3 robot composition

1. Create a V3-only `RobotContainerV3` and V3 bindings package.
2. Call the V3 scheduler from the V3 robot lifecycle path only.
3. Add one V3 teleop behavior and one autonomous-style sequence.
4. Validate priorities, cancellation, disable behavior, and logs on a practice robot.

### Phase 3: Expand selectively

1. Add drivetrain and autonomous integrations only after their library compatibility is demonstrated.
2. Keep V2 and V3 implementations in parallel until their test and on-robot behavior are comparable.
3. Promote V3 from experimental only after a documented release gate is met.

## Release gate for V3 support

Before calling V3 a supported Az-RBSI option, require all of the following:

- The selected WPILib release and V3 vendor dependency are pinned and documented.
- A V3 template starts, enables, disables, and simulates successfully.
- At least one mechanism and the drivetrain have passing unit and simulation tests.
- A real robot completes teleop driving, an autonomous sequence, interruption handling, and disable recovery.
- Required third-party libraries are confirmed V3-compatible or cleanly isolated.
- The V2 template remains unchanged and passes its existing test suite.

Commands V3 is still evolving in the 2027 alpha series; recent releases added gamepad support, edge-trigger factories, and a declarative state-machine API. Keep V3 examples version-pinned and review each alpha release before updating the recommended implementation. [WPILib 2027 change log](https://docs.wpilib.org/en/latest/docs/yearly-overview/yearly-changelog.html)

## Decision rules for teams

- Choose **V2** when preserving a known codebase, using an unported command-returning library, or training a team on established command-based patterns.
- Choose **V3** when beginning a new codebase, willing to work with the 2027-alpha toolchain, and prepared to own migration validation.
- Do not mix V2 and V3 command ownership of a mechanism in the same deployed robot profile.
- Share IO, state, calculations, and tests—not commands, schedulers, or subsystem/mechanism ownership.

## References

- [WPILib Commands V3 `Command` API](https://github.wpilib.org/allwpilib/docs/2027/java/org/wpilib/command3/Command.html)
- [WPILib Commands V3 `Scheduler` API](https://github.wpilib.org/allwpilib/docs/2027/java/org/wpilib/command3/Scheduler.html)
- [WPILib 2027 change log](https://docs.wpilib.org/en/latest/docs/yearly-overview/yearly-changelog.html)
- [WPILib repository: Commands V2 and V3 are both included](https://github.com/wpilibsuite/allwpilib)
- [WPILib Systemcore testing compatibility notes](https://github.com/wpilibsuite/SystemcoreTesting)
