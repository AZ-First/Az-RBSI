# Az-RBSI Pose Buffering

This page contains documentation for the Odometry and Vision Pose Buffering
system included in Az-RBSI.  It is more for reference than instructing teams in
how to do something.

--------

### Background for Need

In previous versions of the Az-RBSI (or the base [AdvantageKit templates](
https://docs.advantagekit.org/getting-started/template-projects)), there is a
temporal disconnect between the "now" state of the drivetrain odometry (wheel
encoders + gyro) and the "delayed" state of vision measurements (by exposure
time, pipeline processing, and network transport).  Essentially, **the
different sensors on the robot do not all report data at the same time**.  This
delay can be 30–120 ms -- which is huge when your robot can move a foot in that
time.  Attempting to correct the "now" odometric pose with a "delayed" vision
estimate introduces systematic error than can cause jitter in the pose estimate
and incorrect downstream computed values.


### What is Pose Buffering

A pose buffer lets you store a time history of your robot’s estimated pose so
that when a delayed vision measurement arrives, you can rewind the state
estimator to the exact timestamp the image was captured, inject the correction,
and then replay forward to the present.  In essence, pose buffers enable **time
alignment between subsystems**.  Since the Az-RBSI assumes input from multiple
cameras and combining that with the IMU yaw queues and high-frequency module
odometry, everything must agree on a common timebase.  Introducing a pose
buffer allows us to query, "What did odometry think the robot pose was at time
`t`?" and compute the transform between two timestamps.  This is the key to
making atency-aware fusion mathematically valid.

Pose buffers dramatically improve **stability and predictability** as well.
They can prevent feedback oscillations caused by injecting corrections at
inconsistent times and enable smoothing, gating, and replay-based estimators.
These are incredibly important for accurate autonomous paths, reliable
auto-aim, and multi-camera fusion.


### Implementation as Virtual Subsystems

The Az-RBSI pose buffer implementation is based on the principle that **Drive
owns the authoritative pose history** via a `poseBuffer` keyed by FPGA
timestamp, and we make sure that buffer is populated using the *same timebase*
as the estimator.  Rather than updating the estimator only “once per loop,”
**all high-rate odometry samples** collected since the last loop are replayed
and inserted into the buffer.

We have a series of three Virtual Subsystems that work together to compute the
estimated position of the robot each loop (20 ms), polled in this order:
* Imu
* DriveOdometry
* Vision

The IMU is treated separately from the rest of the drive odometry because we
use its values in the Accelerometer virtual subsystem to compute the
accelerations the robot undergoes.  Its `inputs` snapshot is refreshed before
odometry replay runs so that during odometry replay, we prefer using the IMU’s
**odometry yaw queue** when it exists and is aligned to the drivetrain odometry
timestamps.  If now, we fall back to interpolating yaw from `yawBuffer` (or
"now" yaw if we have no queue).  This allows for stable yaw-rate gating
(single-camera measurements discarded if the robot is spinning too quickly)
because `yawRateBuffer` is updated in the same timebase as the odometry replay.
When vision asks, "what is the max yaw rate over `[ts-lookback, ts]`," it is
querying a consistent history instead of a mix of current-time samples.

The `DriveOdometry` virtual subsystem drains the PhoenixOdometryThread queues
to get a canonical timestamp array upon which is built the
`SwerveModulePosition` snapshots for each sample index.  The YAW from the IMU
is computed for that same sample time, then the module positions and YAW are
added to the pose estimator using the `.updateWithTime()` function for each
timestamp in the queue.  At the same time, we add the sample to the pose buffer
so that later consumers (vision alignment, gating, smoothing) can ask, "What
did the robot think at time `t`?"  Practically, it runs early in the loop,
drains module odometry queues, performs the `updateWithTime(...)` replay, and
keeps the `poseBuffer`, `yawBuffer`, and `yawRateBuffer` coherent.

Vision measurements get included *after* odometry has advanced and buffered
poses for the relevant timestamps.  Vision reads all camera observations,
applies various gates, chooses one best observation per camera, then fuses them
by picking a fusion time `tF` (newest accepted timestamp), and **time-aligning
each camera estimate from its `ts` to `tF` using Drive’s pose buffer**.  The
smoothed/fused result is then injected through the `addVisionMeasurement()`
consumer in `Drive` at the correct timestamp.  The key is: we never try to
"correct the present" with delayed vision; we correct the past, and the
estimator/pose buffer machinery carries that correction forward coherently.

To guarantee the right computation order, we implemented a simple
**priority/ordering mechanism for VirtualSubsystems** rather than relying on
construction order. Conceptually: `Imu` runs first (refresh sensor snapshot and
yaw queue), then `DriveOdometry` runs (drain odometry queues, replay estimator,
update buffers), then `Vision` runs (query pose history, fuse, inject
measurements), and finally anything downstream (targeting, coordinators, etc.).
With explicit ordering, vision always sees a pose buffer that is current
through the latest replayed timestamps, and its time alignment transform uses
consistent odometry states.


### Relationship Between Pose Buffering and Hardware Control Subsystems

The `DriveOdometry` virtual subsystem exists to **isolate the heavy,
timing-sensitive replay logic** from the rest of `Drive` "control" behavior.
This separation allows `Drive`’s main subsystem to focus on setpoints/commands,
while `DriveOdometry` guarantees that by the time anything else runs, odometry
state and buffers are already up to date for the current cycle.

The pose buffering system sits cleanly between **raw hardware sampling** and
**high-level control**, acting as a time-synchronized "memory layer" for the
robot’s physical motion.  At the bottom, hardware devices are sampled at high
frequency with timestamps measurements in FPGA time and compensation for CAN
latency.  Those timestamped samples are drained and replayed inside
`DriveOdometry`, which feeds the `SwerveDrivePoseEstimator` object. This means
the estimator is not tied to the 20 ms main loop -- it advances according to
the *actual sensor sample times*.  The result is a pose estimate that reflects
real drivetrain physics at the rate the hardware can provide, not just the
scheduler tick rate.

On the control side, everything -- heading controllers, auto-alignment, vision
fusion, targeting -- consumes pose data that is both temporally accurate and
historically queryable.  Controllers still run at the 20 ms loop cycle, but
they operate on a pose that was built from high-rate, latency-compensated
measurements.  When vision injects corrections, it does so at the correct
historical timestamp, and the estimator propagates that correction forward
consistently. The net effect is tighter autonomous path tracking, more stable
aiming, and reduced oscillation under aggressive maneuvers -- because control
decisions are based on a pose model that more faithfully represents the real
robot’s motion and sensor timing rather than a simplified "latest value only"
approximation.
