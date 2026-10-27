# Common Practices of High-Performing FRC Teams

**Research snapshot:** August 29, 2026

**Scope:** recurring practices in 2026 Chief Delphi and Open Alliance material,
with emphasis on teams that demonstrated strong competitive performance,
well-developed engineering systems, or unusually candid post-event analysis.

## Recommendation In Brief

“Top teams all do X” is useful shorthand, but it is not literally true. Strong
teams do not all use swerve, turrets, three cameras, custom scouting software,
or a practice robot. They have different resources and win with different
architectures.

They do repeatedly converge on a more durable set of behaviors:

1. Convert the game into explicit priorities before designing mechanisms.
2. Spend complexity only where it creates a measurable match advantage.
3. Resolve important uncertainty early through prototypes, simulation, and
   integrated testing.
4. Protect time for software, autonomous testing, and driver practice.
5. Treat reliability, serviceability, observability, and fallback operation as
   designed features.
6. Run competition as a disciplined feedback loop: prepare, execute, inspect,
   record, learn, and adapt.

The common denominator is **management of uncertainty**, not a particular
robot archetype. Az-RBSI should encode the technical parts of that discipline
and provide lightweight templates for the human processes around it.

## How To Read The List

Each practice has an adoption classification:

- **Foundation:** broadly transferable and worth treating as a default.
- **Multiplier:** common among strong programs, but its implementation should
  scale to team resources.
- **Conditional:** valuable only when it answers a specific strategic or
  technical need.

The evidence is observational. Open Alliance posts overrepresent teams willing
to publish, and a strong process can be invisible when a team writes mainly
about hardware. This document therefore avoids claiming that a practice is
universal merely because one elite team uses it.

## The Short List

Read each line as “high-performing teams tend to,” not as a universal claim:

### Strategy and design

- Study the whole game before choosing mechanisms.
- Define the robot's intended match role and rank requirements.
- Decline features that do not justify their time, mass, power, and risk.
- Study proven robots and reuse understood ideas.
- Prototype the important unknowns against measurable pass/fail criteria.
- Integrate early enough for mechanisms, wiring, software, and drivers to
  expose one another's problems.
- Track mass, volume, power, schedule, and service clearance with margin.
- Design exposed systems for collision and predictable repair.
- Preserve representative hardware access for programmers and drivers.

### Electrical and software

- Treat wire routing, retention, and motion as mechanical design.
- Standardize connectors, labeling, configurations, firmware, and spares.
- Monitor power, CAN, network, device, and loop health.
- Start production software before the competition robot is complete.
- Maintain authoritative robot state instead of scattered flags.
- Log observations, decisions, outputs, timing, and faults.
- Run production logic in simulation and replay where practical.
- Automate useful execution while retaining practiced manual fallbacks.
- Test autonomous routines against realistic errors and field interactions.
- Keep complexity inside the team's compute, schedule, and maintenance budget.

### Practice and competition

- Protect substantial driver practice and use representative match drills.
- Practice communication, defense, degraded modes, and recovery.
- Use robot-specific pre-match and post-match checklists with named owners.
- Make pit and robot readiness visible to the whole team.
- Record failures before repairing them, then verify and sign off.
- Self-inspect early and after consequential changes.
- Bring verified spares and a configuration/software recovery path.
- Control event changes and retain a known-good rollback.
- Collect scouting data that changes strategy, validate it, and supplement it
  with direct observation.
- Turn scouting into a short, shared match plan with contingencies.
- Document decisions, train backups, run retrospectives, and preserve human as
  well as technical margin.

## 1. Strategy And Scope

### 1.1 Understand the game before inventing the robot

**Classification: Foundation**

Strong teams read the complete manual, enumerate scoring and movement tasks,
play or simulate the game, and test rules knowledge before selecting an
architecture. Team 581 began with the full manual, explicit goals, and
Must/Won't/Could design criteria. Team 7691 used a rules quiz and a human-scale
version of the game. Team 1792 similarly studied the rules and listed every
scoring and acquisition action before discussing mechanisms.

Sources: [Team 581 2026 CAD and code release](https://www.chiefdelphi.com/t/581-blazing-bulldogs-2026-cad-and-code-release/521762),
[Team 7691 Open Alliance thread](https://www.chiefdelphi.com/t/frc-7691-2025-2026-build-thread-open-alliance/506859),
[Team 1792 Open Alliance thread](https://www.chiefdelphi.com/t/1792-round-table-robotics-2026-open-alliance-thread/509877)

**Az-RBSI recommendation:** add a season-planning template containing a rules
quiz link, task inventory, field-zone diagram, and unresolved-rules register.
Keep it outside robot code, but link it from the project documentation.

### 1.2 Define the robot's job and rank requirements

**Classification: Foundation**

Competitive teams distinguish requirements from attractive features. A useful
version is Must/Should/Could/Won't or Needs/Wants/Wishes, tied to the level of
play the team intends to reach. The result should say what the robot must do,
how well it must do it, and what the team has deliberately declined.

Team 6328 summarized its target as a reliable, relatively simple robot that
leaned on its software strengths. Team 5119 explicitly rejected a turret that
did not match its resources and experience, in part to preserve driver-practice
time. That is strategic discipline, not lack of ambition.

Sources: [Team 6328 2026 build thread](https://www.chiefdelphi.com/t/frc-6328-mechanical-advantage-2026-build-thread/509595),
[Team 5119 Open Alliance thread](https://www.chiefdelphi.com/t/frc-5119-team-steam-2026-build-thread-open-alliance/507554),
[2026 technical binder](https://www.chiefdelphi.com/uploads/short-url/aiIYU0Pv6kleLK5yW5mDDAaE2n9.pdf)

**Team routine:** every added feature must name the match advantage, owner,
test, schedule cost, weight cost, power cost, and fallback. If those cannot be
stated, the feature is not ready to enter the plan.

### 1.3 Reuse proven ideas and innovate selectively

**Classification: Foundation**

High-performing teams study past robots and current prototypes. They reuse
known-good geometry, controls, and construction methods, then reserve invention
for the few unknowns that differentiate their strategy. Team 5119 explicitly
looked to successful historical robots rather than reinventing every mechanism;
Team 353 credited designs from teams 6328, 4481, and 2017 while adapting them to
its own package.

Sources: [Team 5119 Open Alliance thread](https://www.chiefdelphi.com/t/frc-5119-team-steam-2026-build-thread-open-alliance/507554),
[Team 353 Open Alliance thread](https://www.chiefdelphi.com/t/pobots-353-2026-open-alliance-build-blog/509221)

This is not “copy without understanding.” Record the source, governing
dimensions, expected operating range, and what changed in the adaptation.

## 2. Design And Mechanical Execution

### 2.1 Prototype the unknown, with a pass/fail question

**Classification: Foundation**

A prototype should answer a decision: required compression, achievable range,
cycle time, impact survival, centering reliability, or packaging envelope.
Team 581 built an early Alpha Bot and several mechanism prototypes, then
discarded a visually plausible dumper archetype when testing showed it was not
consistent enough. Team 4096 moved prototypes into CAD and laser-cut parts to
raise their fidelity and shorten the path into the final design.

Sources: [Team 581 2026 CAD and code release](https://www.chiefdelphi.com/t/581-blazing-bulldogs-2026-cad-and-code-release/521762),
[Team 4096 Open Alliance thread](https://www.chiefdelphi.com/t/frc-team-4096-ctrl-z-2026-build-thread-open-alliance/512198)

**Team routine:** every prototype card should include its question, variables,
measurement method, decision threshold, owner, and deadline. Video without a
decision is documentation, not validation.

### 2.2 Integrate early enough to discover system failures

**Classification: Foundation**

Mechanisms that work alone can still fail because of frame flex, ball flow,
software timing, battery sag, cable motion, collisions, or driver behavior.
Strong teams seek those interactions before competition. Team 5010 delivered a
second robot iteration to software and drive after learning from early testing.
Team 4561 used practice-field cycles to improve both autonomous consistency and
its intake. Team 5667's retrospective provides the inverse lesson: late
mechanical completion left roughly a week and a half for programming, exposed
intake failures, and forced a late climber removal during a weight scare.

Sources: [Team 5010 Open Alliance thread](https://www.chiefdelphi.com/t/frc-5010-tiger-dynasty-2026-build-thread-open-alliance/510219),
[Team 4561 Open Alliance thread](https://www.chiefdelphi.com/t/4561-the-terrorbytes-2026-build-thread-open-alliance/508623),
[Team 5667 season wrap-up](https://www.chiefdelphi.com/t/digital-eagles-5667-2026-build-blog-open-alliance/509725?page=2)

**Team routine:** set a driveable-robot deadline, then a competition-feature
freeze. Protect the interval between them for cycling, autos, tuning, repair,
and practice.

### 2.3 Budget mass, volume, power, and schedule continuously

**Classification: Foundation**

Resource limits are design inputs, not end-of-build surprises. CAD assemblies
should carry realistic materials and masses; electrical design should estimate
simultaneous current; mechanism owners should include wiring, fasteners,
guards, and repair clearance. Team 2102 publicly tracked full-assembly mass and
reserved approximately ten pounds for cable chain, wiring, and later changes
rather than treating the modeled mechanism mass as finished weight.

Source: [Team 2102 2026-2027 Open Alliance thread](https://www.chiefdelphi.com/t/2102-build-thread-2026-2027-open-alliance/521559)

Recommended release gates:

- **Architecture review:** every required function fits in space and power.
- **Detailed-design review:** interfaces, fasteners, wire paths, sensors,
  service access, and realistic mass are present.
- **Manufacturing release:** drawings, stock, tooling, owners, and inspection
  criteria are ready.

### 2.4 Design exposed mechanisms to survive contact

**Classification: Foundation**

Intakes, sensors, cameras, cable runs, and low mechanisms should be evaluated as
collision systems. Use load paths, hard stops, crash bars, compliant elements,
and replaceable wear parts. Teams 2102 and 2826 described explicit crash
protection in their 2026 designs; several post-event reports changed intakes
only after realistic drive and defense exposure.

Sources: [Team 2102 Open Alliance thread](https://www.chiefdelphi.com/t/2102-build-thread-2026-2027-open-alliance/521559),
[Team 2826 Open Alliance thread](https://www.chiefdelphi.com/t/frc-2826-wave-robotics-2026-open-alliance-build-thread/508460)

### 2.5 Design for diagnosis and repair

**Classification: Foundation**

Strong competition robots are not merely durable; predictable failures can be
found and repaired inside a match turnaround. Critical assemblies need visible
inspection points, accessible connectors and fasteners, consistent hardware,
replaceable modules, and staged spares. Add witness marks where fasteners can
move and make wear limits measurable.

**Az-RBSI recommendation:** pair each mechanism class with a documented health
view and pit test command. A replacement should require calibration or homing
through one named workflow, not ad-hoc code edits.

### 2.6 Provide representative hardware access; do not fetishize robot count

**Classification: Multiplier**

Many strong programs build an alpha robot, practice robot, spare mechanisms,
or retain an older chassis. The underlying practice is to let programmers and
drivers work while the competition machine is still being built or serviced.
Team 581 used Alpha, Practice, and Competition robots; Team 5406 kept an
earlier robot available for practice. A lower-resource team can meet the same
goal with a kit chassis, simulator, mechanism rig, or scheduled access blocks.

Sources: [Team 581 2026 CAD and code release](https://www.chiefdelphi.com/t/581-blazing-bulldogs-2026-cad-and-code-release/521762),
[Team 5406 Open Alliance thread](https://www.chiefdelphi.com/t/celt-x-5406-2026-build-thread-open-alliance/511200)

Three robots are conditional. Early, representative access is the transferable
practice.

## 3. Electrical And Controls Hardware

### 3.1 Treat wiring as a mechanical subsystem

**Classification: Foundation**

Electrical reliability comes from connector retention, strain relief, service
loops, abrasion protection, mounting, bend radius, labeling, and planned cable
motion—not merely a correct schematic. Team 5406 responded to a controller
disconnect by improving strain relief and service loops. Team 4322 attributed a
long run without radio, CAN, vision-power, USB, or code failures to a deliberate
electrical-reliability focus, until a main-breaker event exposed the next weak
link.

Sources: [Team 5406 Open Alliance thread](https://www.chiefdelphi.com/t/celt-x-5406-2026-build-thread-open-alliance/511200),
[Team 4322 Open Alliance thread](https://www.chiefdelphi.com/t/4322-clockwork-2026-build-thread-open-alliance/511196)

Required design-review questions:

- Can a connector carry tension, vibrate loose, or be struck?
- Can every high-current termination be inspected and torque-checked?
- Does every moving mechanism have a defined cable path at both limits?
- Can the radio, main controller, switch, and vision system be independently
  identified, rebooted, and replaced?

### 3.2 Standardize, label, inspect, and stock known-good spares

**Classification: Foundation**

Use a small set of connector, wire, fastener, sensor, and controller practices.
Label both ends. Store exported device configurations and known-good firmware
versions. Build critical spares before they are needed and verify them on the
robot or a fixture.

At events, use a fresh, charged, load-tested, tracked battery and inspect the
main power path after every match. A battery-management system can be a sheet
or tags; consistency matters more than sophistication.

### 3.3 Design for power and network degradation

**Classification: Foundation**

Current limits, brownout behavior, CAN utilization, loop timing, camera
bandwidth, controller temperature, and radio health should be visible before
they become driver symptoms. Mechanisms need priorities so a transient load
does not turn into a whole-robot failure.

Team 581 implemented a power manager. Team 6328 reported that an accumulation
of software and vision work caused loop overruns, illustrating that compute and
network budgets must be managed like mass and current.

Sources: [Team 581 2026 CAD and code release](https://www.chiefdelphi.com/t/581-blazing-bulldogs-2026-cad-and-code-release/521762),
[Team 6328 2026 build thread](https://www.chiefdelphi.com/t/frc-6328-mechanical-advantage-2026-build-thread/509595)

## 4. Software And Automation

### 4.1 Start software before the final robot exists

**Classification: Foundation**

Strong programs prepare the repository, dependency versions, simulated IO,
logging, controller mappings, and baseline drive code before mechanisms arrive.
Team 157 had its repository and template ready at kickoff, used simulation in
AdvantageScope, and created early prototype hardware so software could start.

Source: [Team 157 Open Alliance thread](https://www.chiefdelphi.com/t/frc-157-the-aztechs-2026-build-thread-open-alliance/509425)

**Az-RBSI recommendation:** a generated project should boot in simulation,
exercise every configured subsystem, and show expected telemetry before any
CAN device is connected.

### 4.2 Keep one authoritative state model

**Classification: Foundation**

The robot should have authoritative sources for pose, mechanism position,
match state, and current intent. Commands consume that state instead of
reconstructing it through scattered booleans. Global field localization should
support broad navigation; a tag-relative or mechanism-local observation can
take over for the last precise alignment when appropriate.

Teams 6328 and 1360 have described that global-plus-local pattern. It reduces
dependence on a globally perfect estimate while retaining field-wide autonomy.

Sources: [Team 6328 localization discussion](https://www.chiefdelphi.com/t/frc-6328-mechanical-advantage-2025-build-thread/476206),
[Team 1360 Open Alliance thread](https://www.chiefdelphi.com/t/frc-1360-2026-build-thread/507908)

See [Pose, Odometry, and Vision Integration](Pose-Odometry-Vision-Integration.md)
for the Az-RBSI implementation details.

### 4.3 Log facts, decisions, and health—not just setpoints

**Classification: Foundation**

Top-level debugging depends on reconstructing what the robot observed and why
it acted. Log sensor inputs, outputs, pose observations and acceptance reasons,
command transitions, driver requests, safety inhibits, loop timing, power, and
faults. Team 9280 describes logs as the source of truth and uses the same
software paths for real operation, simulation, and replay.

Source: [Team 9280 2026 Open Alliance software post](https://www.chiefdelphi.com/t/high-altitude-9280-build-thread-2026-open-alliance/509708?page=2)

**Az-RBSI recommendation:** define a stable telemetry contract and a standard
AdvantageScope layout. A pit crew should be able to answer “sensor, wiring,
configuration, mechanism, or decision logic?” from one match log.

### 4.4 Automate execution while preserving obvious fallbacks

**Classification: Foundation**

Driver assists should aim, stage, coordinate, protect, and recover mechanisms,
but a failed camera or estimator must not make the robot unusable. Team 6328
added fixed-distance presets and a force-launch control after vision and
auto-aim failures at Week Zero. Team 581 provided preset-shot and dashboard
fallbacks alongside more advanced automation.

Sources: [Team 6328 override controls](https://www.chiefdelphi.com/t/frc-6328-mechanical-advantage-2026-build-thread/509595/606),
[Team 581 2026 CAD and code release](https://www.chiefdelphi.com/t/581-blazing-bulldogs-2026-cad-and-code-release/521762)

Every automated action should define:

- its required sensors and confidence;
- a timeout and interruption path;
- a lower-capability fallback;
- a direct driver override; and
- a logged explanation of why it was blocked or changed.

See [Robot Decision Automation](Robot-Decision-Automation.md) for the proposed
intent, world-state, and superstructure model.

### 4.5 Test the same code in simulation, replay, and hardware

**Classification: Multiplier**

Simulation is most valuable when it runs production commands and subsystem
logic, not a parallel demonstration. Replay is most valuable when logged inputs
can reproduce a decision without energizing hardware. Pair both with hardware
tests for timing, friction, impacts, radio behavior, and sensor limitations.

Do not confuse a convincing field animation with validation. Maintain explicit
acceptance tests for pose error, cycle time, shot error, autonomous completion,
recovery, and degraded modes.

### 4.6 Keep complexity inside measured compute and maintenance budgets

**Classification: Foundation**

Advanced teams remove features that cost more reliability or practice than they
return in points. Profile periodic-loop time, network load, log volume, garbage
collection, and vision processing. Prefer bounded algorithms, explicit state,
and inspectable commands over opaque layers that only one student understands.

Repository discipline supports this: use version control, review important
changes, protect the deployable branch, tag event releases, and retain a known-
good rollback point. The goal is quick recovery, not ceremony.

### 4.7 Test autonomous routines on a representative field

**Classification: Foundation**

An autonomous routine is not complete when it traces correctly once. Test
start-pose error, game pieces, boundaries, bumps, battery variation, missed
acquisition, obstruction, and interruption. Team 4561 attributed a consistent
2.5-cycle autonomous routine to practice-field testing. Team 581 added
automatic unbeaching so a field interaction did not necessarily end the rest
of the routine.

Sources: [Team 4561 Open Alliance thread](https://www.chiefdelphi.com/t/4561-the-terrorbytes-2026-build-thread-open-alliance/508623),
[Team 581 2026 CAD and code release](https://www.chiefdelphi.com/t/581-blazing-bulldogs-2026-cad-and-code-release/521762)

## 5. Driver Development

### 5.1 Protect substantial, purposeful drive practice

**Classification: Foundation**

Strong teams do not treat practice as whatever time remains. They schedule it,
provide field access, measure repeatable tasks, and feed driver observations
back into mechanical and software changes. Team 4096 reported that dedicated
code and drive practice exposed strengths and weaknesses that shaped its next
robot iteration. Team 1730 hosted “Practice with a Purpose” on a shared full
field, and Team 360 described community collaboration around a full practice
field.

Sources: [Team 4096 Open Alliance thread](https://www.chiefdelphi.com/t/frc-team-4096-ctrl-z-2026-build-thread-open-alliance/512198),
[Team 1730 Open Alliance thread](https://www.chiefdelphi.com/t/1730-team-driven-2026-build-thread-open-alliance/508101),
[Team 360 Open Alliance thread](https://www.chiefdelphi.com/t/frc-360-the-revolution-2026-build-thread-open-alliance/510290)

Practice should include:

- repeatable acquisition, scoring, traversal, and endgame drills;
- full match cycles with realistic traffic and game-piece depletion;
- defense from and against the robot;
- degraded vision, failed automation, jams, and manual recovery;
- autonomous-to-teleop handoff and controller reconnects; and
- concise driver feedback after each block.

### 5.2 Make controls stable, sparse, and explainable

**Classification: Foundation**

Essential controls should fit the drive team's practiced mental model. Prefer
high-level intent controls, consistent physical locations, tactile separation,
and deliberate confirmation for dangerous actions. Avoid remapping controls at
an event unless the expected benefit exceeds the new execution risk.

Az-RBSI controller APIs should name inputs by physical position while robot
bindings name the intent. This preserves reusable controller code without
hiding the action assigned in a particular robot project.

### 5.3 Practice communications and contingencies as part of driving

**Classification: Foundation**

The drive team should have short calls for role changes, defense, passing,
failed mechanisms, endgame timing, and emergency disable. Pre-match strategy
must identify the alliance plan, starting locations, autonomous interactions,
traffic lanes, likely defense, and what changes if a partner or mechanism
fails.

## 6. Competition Operations

### 6.1 Use specific pre-match and post-match checklists

**Classification: Foundation**

Memory is unreliable under event pressure. A checklist should be ordered,
specific to the robot, practiced, and updated after failures. It should assign
who inspects structure, mechanisms, electrical, software, battery, and final
functional state.

Team 7691's 2026 pit system records issues and fixes, divides post-match work
among programming, electrical, and mechanical roles, then performs a power
cycle, system check, and signoff. Chief Delphi's 2026 checklist discussion also
emphasized concrete inspection items and evolving the list from observed
failures.

Sources: [Team 7691 pit-system post](https://www.chiefdelphi.com/t/frc-7691-2025-2026-build-thread-open-alliance/506859?page=3),
[pre-match checklist discussion](https://www.chiefdelphi.com/t/pre-match-checklists/473008)

Minimum post-match loop:

1. Isolate and power down safely.
2. Record driver symptoms before changing anything.
3. Inspect known wear and impact points.
4. Review faults and the relevant log interval.
5. Repair with one accountable owner.
6. Power, calibrate, and run a short functional test.
7. Install and record the next battery.
8. Obtain a final ready/not-ready signoff.

### 6.2 Make robot and pit status visible

**Classification: Foundation**

Everyone should know whether the robot is safe to approach, under repair,
awaiting software, ready for functional test, or ready to queue. Team 7691 used
a color-coded pit status and a digital system for schedule, notes, checklists,
parts, batteries, and break logs. The exact app is conditional; a visible state
and one responsible coordinator are not.

Source: [Team 7691 pit-system post](https://www.chiefdelphi.com/t/frc-7691-2025-2026-build-thread-open-alliance/506859?page=3)

### 6.3 Self-inspect early and keep the robot inspection-ready

**Classification: Foundation**

Use the current official checklist before load-in, after consequential changes,
and before leaving the shop for the next event. Ask for pre-inspection when the
event permits it. Keep evidence such as component documentation accessible.
PRECHECK was offered in 2026 as a friendlier self-inspection interface with
links to the underlying rules.

Source: [2026 inspection-checklist discussion](https://www.chiefdelphi.com/t/2026-team-update-07/513642)

### 6.4 Record failures before fixing them

**Classification: Foundation**

Capture the match, timestamp, symptom, driver observation, fault/log evidence,
root cause, repair, and verification. This converts event chaos into reliability
engineering and prevents the same symptom from receiving a different guess
each time.

Use a short post-match debrief, not a long design meeting. Schedule deeper
analysis away from queue pressure.

### 6.5 Bring staged spares, tools, and configuration recovery

**Classification: Multiplier**

Prioritize spares by probability, consequence, and repair time. Complete
mechanism or wiring modules are valuable when they reduce turnaround, but
unverified spares can create a second failure. Include controller configuration,
calibration data, deployable software, firmware, and network settings in the
recovery kit.

### 6.6 Control event changes

**Classification: Foundation**

At competition, define the problem before modifying the robot. Change one
causal variable when practical, record it, test it, and retain a rollback path.
Risky feature work should not displace a functioning match configuration
without an explicit strategy decision.

## 7. Scouting And Match Strategy

### 7.1 Collect the smallest trustworthy dataset that changes decisions

**Classification: Foundation**

Strong scouting systems are designed around match strategy and alliance
selection, not around collecting every observable fact. Combine quantitative
production with qualitative observations such as preferred paths, failure
modes, defense behavior, recovery, driver control, and compatibility.

Team 9483—drawing on experience at high levels of play—described pre-scouting
prior matches, combining pit and match scouting, and managing scout fatigue and
data quality. Team 5572's 2026 retrospective similarly identified training,
rotation, missing submissions, and data freshness as operational constraints.

Sources: [Team 9483 scouting thread](https://www.chiefdelphi.com/t/9483-scouting-saga-from-vibes-and-paper-to-predictive-data/521985),
[Team 5572 scouting overview](https://www.chiefdelphi.com/t/frc-5572-scouting-system-data-and-strategy-overview-2026/519045)

### 7.2 Validate data and retain human observations

**Classification: Foundation**

Automated rankings, EPA, and dashboards are inputs, not decisions. Flag
impossible values, missing matches, defense-distorted data, and role changes.
Watch candidates directly before alliance selection and record evidence for
disagreements. An alliance partner should be selected for the intended playoff
role and compatibility, not merely the largest aggregate number.

### 7.3 Collaborate when it improves coverage or quality

**Classification: Multiplier**

Scouting alliances can reduce labor and improve coverage if teams agree on
definitions, training, validation, data ownership, and failure recovery. A 2026
Archimedes alliance used shared ScoutRadioz forms and a second pit-scouting
round before alliance selection. Team 1676 also maintained a public 2026
Championship pre-scouting database to avoid duplicated work.

Sources: [2026 Archimedes scouting alliance](https://www.chiefdelphi.com/t/2026-archimedes-scouting-alliance/519248),
[2026 Championship pre-scouting database](https://www.chiefdelphi.com/t/2026-world-championship-public-pre-scouting-database/519278)

### 7.4 Turn scouting into an executable match plan

**Classification: Foundation**

Before every match, agree on autonomous positions, collision risks, scoring or
passing roles, traffic lanes, expected hub timing, defensive assignments,
endgame responsibilities, and contingency calls. The output must be short
enough for all three drive teams to remember.

## 8. Team Process And Learning

### 8.1 Assign ownership without creating knowledge silos

**Classification: Foundation**

Every deliverable and event role needs an owner, a reviewer or backup, a due
date, and an acceptance condition. Train at least one backup for deploy,
electrical diagnosis, calibration, drive operation, scouting administration,
and critical repairs.

### 8.2 Document decisions and publish enough to be accountable

**Classification: Multiplier**

Design reviews, technical binders, code releases, and Open Alliance posts force
teams to explain what they chose and what happened. Team 1710 published code,
CAD, and a technical binder; many of the strongest 2026 sources used here also
included candid failure reports rather than only reveal videos.

Source: [Team 1710 resources](https://www.chiefdelphi.com/t/frc-1710-resources/520754)

Public release is conditional. Internal decision records, setup instructions,
and post-event retrospectives are foundational.

### 8.3 Run short feedback loops

**Classification: Foundation**

After prototypes, practice blocks, matches, and events, ask:

1. What outcome did we expect?
2. What actually happened?
3. What evidence distinguishes possible causes?
4. What is the smallest useful change?
5. How will we verify it and prevent regression?

Keep the record blame-free and technically specific. A failure found during
practice is useful schedule information; the same failure rediscovered in
eliminations is process debt.

### 8.4 Preserve margin and team sustainability

**Classification: Foundation**

Time, weight, power, money, attention, and student energy are all finite.
Schedule meals and rest at events, rotate scouts, constrain late-night heroics,
and maintain technical scope the team can explain and repair. Sustainable
execution across an event is a competitive capability.

## Practices That Top Teams Do **Not** All Share

Do not convert visible correlation into a universal requirement. The source
material does not support “top teams all” doing any of the following:

- building two or three complete robots;
- using swerve rather than tank drive;
- using a turret or shooting while moving;
- using a particular motor, camera, controller, or vendordep;
- writing a custom scouting application;
- using full-field global localization for every precision action;
- maximizing mechanism count;
- using one command framework or software architecture; or
- having a large team, large shop, or large budget.

The transferable lesson behind most of these is access, testability,
observability, strategic fit, or reduced cognitive load. Implement that lesson
at the scale the team can sustain.

## What Az-RBSI Should Encode

The following should become opinionated defaults or supported templates in the
RBSI toolchain:

| Area | Recommended RBSI Capability |
| --- | --- |
| Bring-up | Simulated boot, subsystem self-tests, device/configuration inventory, and explicit calibration status. |
| Reliability | Health signals for power, CAN, loop timing, controllers, cameras, sensors, and mechanism progress. |
| Observability | Stable log schema, command/intent transitions, vision accept/reject reasons, faults, and standard AdvantageScope layouts. |
| Degraded operation | Named sensor dependencies, timeouts, preset/manual fallbacks, driver overrides, and dashboard disable switches. |
| Autonomous | Start-pose validation, path/field overlays, event-marker logging, interruption tests, and realistic simulation/replay. |
| Controls | Physical controller names in reusable controller classes; semantic intent names in robot bindings. |
| Pit support | Safe mechanism test commands, calibration workflow, configuration export/restore, and a one-page health summary. |
| Process templates | Strategy priorities, prototype card, design-review checklist, software release record, pre/post-match checklist, and failure report. |

Az-RBSI should **not** prescribe a competitive archetype, mechanism count,
vendor ecosystem, scouting platform, or number of robots. Those are team and
game decisions.

## Suggested Adoption Sequence

### Before kickoff

1. Establish repository, simulation, logging, deployment, and rollback.
2. Build controller, battery, wiring, inspection, and pit-checklist standards.
3. Train backups and rehearse diagnosis on an existing robot.
4. Prepare strategy, prototype, design-review, and retrospective templates.

### Weeks 1–2

1. Complete game analysis and prioritized requirements.
2. Prototype only the decisions that can change architecture.
3. Select a deliberately bounded robot role.
4. Release chassis and mechanism interfaces with mass and service margin.

### Weeks 3–4

1. Integrate the first driveable system.
2. Give software and drivers representative access.
3. Establish baseline autos and manual fallbacks.
4. Start full-system reliability cycling and log review.

### Before the first event

1. Freeze competition functionality early enough to practice it.
2. Run full matches, defense, degraded modes, and repair turnarounds.
3. Self-inspect and verify spares, batteries, configuration recovery, and
   checklists.
4. Train scouts and rehearse the pre-match strategy workflow.

### At every event

1. Execute the pre-match plan and checklist.
2. Capture symptoms and logs immediately after the match.
3. Inspect, repair, test, and sign off through named roles.
4. Update scouting and strategy from current evidence.
5. Make controlled changes with a rollback path.

## Bottom Line

The strongest recurring 2026 pattern is not “build the most advanced robot.”
It is: **make explicit choices, expose uncertainty early, preserve practice
time, measure the real system, degrade gracefully, and learn faster than the
event schedule changes.**

Az-RBSI can help by making a well-instrumented, testable, recoverable robot the
easy default. The team must supply the strategic restraint and operational
discipline that turn those capabilities into match performance.
