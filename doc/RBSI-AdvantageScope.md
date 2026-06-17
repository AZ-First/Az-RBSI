# RBSI AdvantageScope Layout

This page describes `AdvantageScope RBSI Standard.json`, the standard
AdvantageScope layout included with RBSI.

Use this layout when reviewing logs from practice, matches, replay, and pit
debugging. It is intentionally generic: it focuses on drivetrain, odometry,
vision, power, CAN health, loop timing, and the example flywheel.

## Loading The Layout

1. Open AdvantageScope.
2. Open a live robot connection or a `.wpilog`.
3. Use AdvantageScope's layout import/open command.
4. Select `AdvantageScope RBSI Standard.json` from the repository root.

The companion `AdvantageScope Swerve Calibration.json` layout is narrower and
intended for swerve setup and calibration.

## Tabs

### 2D Field - Odometry

Shows `/RealOutputs/Odometry/Robot` on the 2026 field. Use this first when
checking whether the robot pose moves correctly from wheel odometry and gyro
alone.

### 2D Field - Vision

Overlays robot odometry, visible AprilTag poses, and accepted camera robot-pose
observations. Use this to confirm camera transforms and tag filtering.

### Vision Acceptance

Plots per-camera observation counts and vision trust diagnostics:

- observations seen, accepted, and rejected,
- tag count this loop,
- average tag distance,
- linear and angular standard deviations,
- accepted/fused boolean stripes.

Use this tab when vision appears connected but pose is not updating.

### 2D Field - Replay

Shows replay-output odometry and replay-output vision observations. Use this
when running AdvantageKit replay to compare simulated/replayed pose behavior
against real match data.

### Real vs Replay

Overlays real and replay robot poses. This is useful when validating replay
changes to odometry, pose fusion, or autonomous behavior.

### Odometry Health

Plots odometry support signals:

- IMU latency seconds,
- Phoenix odometry dropped samples,
- Spark odometry dropped samples,
- pose reset timestamp,
- pose reset epoch.

Use this tab when pose updates stutter, replay diverges, or vision appears to be
rejected after pose resets.

### Disabled Pose Fusion

Plots disabled coast and disabled vision-fusion behavior:

- disabled coast active,
- stationary loop count,
- max wheel delta in meters,
- yaw rate in radians per second,
- disabled vision blend alpha,
- disabled vision reject/init-snap booleans.

Use this when the robot is disabled on the field and vision is expected to pull
pose gently toward tag observations after the robot stops coasting.

### Swerve

Shows measured, setpoint, and optimized swerve module states. Use this for
module direction, optimization, and drive command debugging.

### Drive Closed Loop

Plots drive closed-loop support signals where available:

- drive velocity in radians per second,
- closed-loop drive velocity in radians per second,
- drive acceleration,
- feedforward voltage,
- battery voltage.

Some signals are produced only by specific IO implementations. For example,
Phoenix and Spark paths do not log every identical helper key.

### Power (V / A)

Separates voltage and current onto different axes:

- PDH/PDP voltage,
- total current,
- drive current,
- steer current,
- example flywheel current,
- brownout-imminent stripe.

Use this during pit checks to find low battery voltage, current spikes, or an
unexpectedly expensive mechanism.

### Battery Usage

Plots higher-level battery estimates:

- battery percent estimate,
- amp-hours used,
- total power,
- energy in watt-hours and joules,
- brownout-imminent stripe.

Use this as a trend view across practice runs or long troubleshooting sessions.

### CAN Health (utilization / counts)

Separates CAN utilization from error counts:

- roboRIO bus utilization,
- drivetrain CAN bus utilization,
- receive/transmit error counts,
- bus-off counts,
- TX full count,
- enabled stripe.

Utilization is locked to a `0..1` range because CTRE reports it as a fraction.
Error counts are left unlocked so spikes are visible.

### Loop Time - Robot (ms)

Plots robot-level loop timing in milliseconds:

- full robot cycle,
- user code,
- log periodic,
- garbage collection,
- RBSI code-loop timing split into virtual subsystem and command scheduler
  portions.

Use this first when chasing 20 ms hot-loop issues.

### Loop Time - Mechanisms (ms)

Plots `RBSISubsystem` timing in milliseconds. The template includes:

- `DriveMS`,
- `FlywheelMS`.

Add mechanism subsystem timing here as you create robot-specific mechanisms.

### Loop Time - Virtual (ms)

Plots `VirtualSubsystem` timing in milliseconds:

- `ImuMS`,
- `DriveOdometryMS`,
- `VisionMS`,
- `AccelerometerMS`,
- `RBSICANHealthMS`,
- `RBSIPowerMonitorMS`.

Use this when virtual processing, vision, or health monitoring is suspected of
slowing the main loop.

### 3D Field

Shows 3D robot and vision objects. Use this to spot obvious transform mistakes
such as cameras below the floor, tags in the wrong place, or the robot rotated
incorrectly.

### Video

Reserved for AdvantageScope video sync. Use it when match video is available
and aligned with a `.wpilog`.

## Adding Mechanisms To Power

Power logging is automatic only after the mechanism reports its PDH/PDP ports
and is passed to `RBSIPowerMonitor`.

1. Add mechanism device IDs and power ports in `Constants.RobotDevices`.

   ```java
   public static final RobotDeviceId ARM_LEADER = new RobotDeviceId(20, CANBuses.RIO, 5);
   public static final RobotDeviceId ARM_FOLLOWER = new RobotDeviceId(21, CANBuses.RIO, 6);
   ```

2. In the mechanism IO implementation, return those ports.

   ```java
   @Override
   public int[] powerPorts() {
     return new int[] {
       RobotDevices.ARM_LEADER.getPowerPort(),
       RobotDevices.ARM_FOLLOWER.getPowerPort()
     };
   }
   ```

3. In the subsystem, forward IO power ports through `getPowerPorts()`.

   ```java
   @Override
   public int[] getPowerPorts() {
     return io.getPowerPorts();
   }
   ```

4. In `RobotContainer`, pass the subsystem to `RBSIPowerMonitor`.

   ```java
   m_power = new RBSIPowerMonitor(batteryCapacity, m_flywheel, m_arm);
   ```

5. In AdvantageScope, add the logged key to `Power (V / A)`:

   ```text
   /RealOutputs/Power/Subsystems/Arm_Current
   ```

The subsystem name comes from `getClass().getSimpleName()`. If the class is
`Arm`, the power key is `Arm_Current`. If the class is `CoralIntake`, the key is
`CoralIntake_Current`.

## Adding Mechanisms To Loop Timing

Mechanism timing is automatic for classes that extend `RBSISubsystem` and put
their periodic work in `rbsiPeriodic()`.

1. Make the mechanism extend `RBSISubsystem`.

   ```java
   public class Arm extends RBSISubsystem {
     @Override
     protected void rbsiPeriodic() {
       io.updateInputs(inputs);
       Logger.processInputs("Arm", inputs);
     }
   }
   ```

2. Do not override `periodic()`. RBSI owns `periodic()` so it can time every
   subsystem consistently.

3. After deploying, add the mechanism timing key to `Loop Time - Mechanisms
   (ms)`:

   ```text
   /RealOutputs/LogPeriodic/Subsystem/ArmMS
   ```

Again, the key uses the Java class simple name. `CoralIntake` becomes:

```text
/RealOutputs/LogPeriodic/Subsystem/CoralIntakeMS
```

Virtual subsystems use the same pattern but appear under:

```text
/RealOutputs/LogPeriodic/VirtualSubsystem/<ClassName>MS
```

Only put robot-wide observer/coordination code in a `VirtualSubsystem`.
Motor-owning mechanisms should normally be `RBSISubsystem` classes so WPILib
command requirements work correctly.

## Related Pages

- [RBSI-Drive.md](RBSI-Drive.md): drivetrain and odometry signals.
- [RBSI-Vision.md](RBSI-Vision.md): vision logging and filtering.
- [RBSI-PoseBuffer.md](RBSI-PoseBuffer.md): pose timing and replay behavior.
- [RBSI-SysId.md](RBSI-SysId.md): characterization logs and mechanism tuning.
