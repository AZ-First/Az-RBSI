# Commands V3 branch

`sc2027_cv3` uses the 2027 alpha-6 Commands V3 library and the V3 `Scheduler`,
`Mechanism`, `Command`, and `Trigger` APIs. Teleop drive, Autopilot drive-to-pose,
the example flywheel, and the custom drive feedforward and wheel-radius
characterization commands use V3 coroutines. `Robot.robotPeriodic()` runs the V3
scheduler on the robot loop thread. `RBSISubsystem` registers its periodic and
simulation callbacks with that scheduler.

PathPlanner autos and WPILib SysId routines are temporarily disabled on this
branch. The pinned PathPlanner library returns V2 commands and requires V2
subsystems; the installed V3 alpha has no `SysIdRoutine`. Choreo remains
unavailable pending a compatible 2027 library. Their previous integration code
and tests remain in place as `Commands V3 deferred` comments to make a later
port easier. The old PathPlanner and Commands V2 vendordeps are preserved as
`.json.disabled` files; rename or replace them only when compatible libraries
are available. `AutoType.MANUAL` is the supported setting. The autonomous
chooser currently offers a do-nothing default and the two custom drive
characterization commands. Replace or extend the manual chooser options in
`RobotContainer` for a competition autonomous routine.

The PathPlanner deploy files are retained for a future V3-compatible integration,
but they do not run on this branch. The general autonomous and SysId guides in
this repository describe the V2 branch.

Use the bundled WPILib Java 25 JDK to build and test:

```sh
JAVA_HOME=/path/to/wpilib/2027_alpha5/jdk ./gradlew test --offline
```

The Gradle test task opens JDK continuation access needed by V3. GradleRIO
already supplies those JVM options for robot deployment and simulation.
