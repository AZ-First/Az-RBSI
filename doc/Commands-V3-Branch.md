# Commands V3 branch

`commands_v3` is rebased onto `alpha8` and uses the 2027 alpha-8 Commands V3 library and the V3 `Scheduler`,
`Mechanism`, `Command`, and `Trigger` APIs. Teleop drive, Autopilot drive-to-pose,
the example flywheel, and the custom drive feedforward and wheel-radius
characterization commands use V3 coroutines. `Robot.robotPeriodic()` runs the V3
scheduler on the robot loop thread. `RBSISubsystem` registers its periodic and
simulation callbacks with that scheduler.

PathPlanner autos and WPILib SysId routines are temporarily disabled on this
branch. The pinned PathPlanner library returns V2 commands and requires V2
subsystems; Commands V3 alpha-8 has no `SysIdRoutine`. ChoreoLib 2027 alpha-3
also imports Commands V2, so its vendordep and `ChoreoAutoController.java` are
preserved with `.disabled` suffixes. The previous PathPlanner, Choreo, and
SysId integration remains in `Commands V3 deferred` comments for later ports.
Re-enable those dependencies only when V3-compatible releases are available,
then adapt their command and mechanism interfaces and restore their tests.

YAGSL and YAMS now have Commands V3 vendordeps, and the project builds with
both installed. The YAGSL runtime path is still disabled in `SwerveConstants`:
the checked-in `src/main/deploy/swerve` files use the old JSON format, and
`swervedrive.json` selects a roboRIO `navx_spi` gyro. The 2027 YAGSL parser
requires regenerated JSON; the gyro choice must also be updated for the
Systemcore hardware before restoring `YagslConstants` and its mapping into
`SwerveConstants`. Until then, `SwerveType.PHOENIX6` remains the configured
drive type.

`AutoType.MANUAL` is the supported setting. The `LoggedNetworkChooser` currently
offers a do-nothing default and the two custom drive characterization commands.
Replace or extend the manual chooser options in `RobotContainer` for a
competition autonomous routine. `RBSISubsystem` implements the alpha-8
`Mechanism` interface and retains its timed periodic callback registration.

The PathPlanner deploy files are retained for a future V3-compatible integration,
but they do not run on this branch. The general autonomous and SysId guides in
this repository describe the V2 branch.

Use the bundled WPILib Java 25 JDK to build and test:

```sh
JAVA_HOME=/path/to/wpilib/2027_alpha7/jdk ./gradlew test --offline
```

The Gradle test task opens JDK continuation access needed by V3. GradleRIO
already supplies those JVM options for robot deployment and simulation.
