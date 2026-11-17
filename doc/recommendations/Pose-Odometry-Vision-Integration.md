# Pose, Odometry, and Vision Integration Recommendations

**Research snapshot:** August 29, 2026

**Scope:** Az-RBSI's 2027-alpha pose pipeline, 2026 Chief Delphi and Open
Alliance reports, and localization-related libraries available through
vendordeps.

## Executive Recommendation

Az-RBSI has a strong low-level odometry foundation, but its custom vision
pre-fusion layer should be simplified.

Keep the high-rate, timestamped wheel and gyro sampling, bounded queues,
AdvantageKit logging, simulation/replay support, and one authoritative WPILib
`SwerveDrivePoseEstimator`. Change the vision path so every accepted camera
measurement is validated and then submitted independently, in timestamp order,
to that estimator. Remove the rolling vision smoother and the custom
inverse-variance fusion of already-solved camera poses.

The highest-priority work is:

1. reject non-finite, stale, future, and implausibly discontinuous observations;
2. fix Limelight MegaTag2 selection and supply it raw gyro heading rather than
   the vision-corrected estimator heading;
3. remove correlated pre-fusion and rolling reuse of old vision frames;
4. use WPILib's estimator history as the single latency-compensation mechanism;
5. make gyro-loss and odometry-sample mismatch behavior real, explicit, and
   tested; and
6. introduce a vendor-neutral `LocalizationIO` boundary so PhotonVision,
   Limelight, QuestNav, and future sources produce the same validated
   observation type.

PhotonLib and WPILib should remain the default portable implementation. For an
all-CTRE drivetrain, Phoenix 6's generated swerve estimator is a reasonable
alternative backend. QuestNav is worth an experimental adapter, but not as the
only localization source. YAGSL and MapleSim can reduce implementation effort
for particular teams, but Az-RBSI should not stack multiple competing owners of
swerve odometry on the same robot.

## Current Az-RBSI Data Flow

The current pipeline is:

```text
Phoenix/Spark sampling threads       IMU virtual subsystem
             |                              |
             +---------- timestamped queues+
                                            |
                                    DriveOdometry
                           updateWithTime() for every sample
                                            |
                           WPILib SwerveDrivePoseEstimator
                                            |
                              external pose/yaw buffers
                                            |
          PhotonVision or Limelight -> Vision gates
                                            |
                         best observation per camera
                                            |
                    align cameras to newest timestamp
                                            |
                       inverse-variance camera fusion
                                            |
                       250 ms rolling pose smoothing
                                            |
                     bounded Drive vision-input queue
                                            |
                   estimator update on the next loop
```

The relevant implementation is in
[`DriveOdometry.java`](../../src/main/java/frc/robot/subsystems/drive/DriveOdometry.java),
[`Drive.java`](../../src/main/java/frc/robot/subsystems/drive/Drive.java), and
[`Vision.java`](../../src/main/java/frc/robot/subsystems/vision/Vision.java).

## What Az-RBSI Already Does Well

### Precision

- Wheel positions and Pigeon yaw are sampled above the 20 ms robot-loop rate.
- `DriveOdometry` calls `updateWithTime(...)` for each historical sample instead
  of collapsing a batch to its newest value.
- CAN latency is considered when building Phoenix sample timestamps.
- Camera observations carry capture timestamps and per-measurement standard
  deviations.
- Uncertainty already scales with distance squared, tag count, camera quality,
  observation type, and a configurable trusted-tag policy.

This matches the strongest consensus in 2026 discussions: continuous wheel and
gyro odometry should provide responsive tracking, while timestamped absolute
measurements correct drift. A Chief Delphi discussion about four-camera systems
specifically recommends feeding each estimate and its own uncertainty into one
filter rather than maintaining one estimator per camera. See
[PhotonVision questions](https://www.chiefdelphi.com/t/photonvision-questions/518082).

### Reliability

- Sensor IO is replayable through AdvantageKit.
- Pose resets increment an epoch, preventing pre-reset camera frames from being
  accepted later.
- Camera timestamps must be monotonic per camera.
- Single-tag ambiguity, Z, field bounds, and high-yaw single-tag observations
  are gated.
- Odometry and vision queues are bounded and overwrite old data rather than
  allowing unbounded latency.
- Pose estimation continues while disabled, and autonomous start has an
  explicit vision-freshness and position-consistency policy.

These are materially better safeguards than many 2026 examples, where the
dominant failure was simply submitting a camera pose with the current robot
time. In one representative case, correcting the measurement timestamp fixed
the integration issue. See
[Estimating Pose with CTRE Swerve and PhotonVision](https://www.chiefdelphi.com/t/estimating-pose-with-ctre-swerve-template-and-photonvision-is-inaccurate/515230).

### Efficiency and Diagnosis

- Camera IO drains unread results but retains only the newest result under
  backlog.
- Estimator work and vision application are bounded per loop.
- Bulk Phoenix telemetry refresh avoids repeated calls.
- Lock wait, estimator time, vision time, queue depth, and dropped samples are
  logged.
- The project already supports real, simulation, and replay IO paths.

This aligns with Open Alliance teams treating logs as a flight recorder for
odometry, slip, latency, and regression analysis. See
[High Altitude 9280's 2026 logging and replay plan](https://www.chiefdelphi.com/t/high-altitude-9280-build-thread-2026-open-alliance/509708?page=2).

## Findings and Required Changes

### P0: Validate Every Observation Before It Can Reach Math Code

`Vision.passesScrutiny(...)` does not currently reject non-finite timestamps,
pose components, ambiguity, or distance. Java comparisons with `NaN` usually
evaluate false, so a NaN pose can pass the Z and field-bound checks and poison
fusion or the estimator. A 2026 Chief Delphi report documents pose output
becoming NaN and explicitly recommends rejecting non-finite vision data in
addition to updating camera firmware. See
[CTRE Tuner X Swerve Position Returning NaN](https://www.chiefdelphi.com/t/ctre-tuner-x-swerve-position-returning-nan-not-a-number/519473).

The custom `ConcurrentTimeInterpolatableBuffer.getSample(...)` also clamps a
request outside its time range to the oldest or newest pose. That behavior is
convenient for generic interpolation but unsafe for camera alignment: an old or
future camera timestamp can silently be aligned using the wrong endpoint.

Add hard gates for:

- finite timestamp, pose, ambiguity, average distance, and standard deviations;
- timestamp inside estimator-history coverage;
- maximum receipt age and maximum future-clock skew;
- known AprilTag IDs from the active field layout;
- plausible camera-to-tag distance; and
- innovation relative to the estimator pose sampled at the observation time.

For the first implementation, use a configurable translational and angular
innovation gate. WPILib and CTRE recommend accepting vision only when it is
already roughly within one meter of the current estimate; CTRE documents this
directly in its
[swerve estimator API](https://api.ctr-electronics.com/phoenix6/latest/cpp/classctre_1_1phoenix6_1_1swerve_1_1impl_1_1_swerve_drivetrain_impl.html).
Later, the gate can be normalized by the observation covariance. Always permit
a separate, explicit initialization path when no trustworthy field pose exists.

Log a rejection enum and residual values for every source. Counts alone are not
enough to determine why localization disappeared.

### P0: Fix the Limelight MegaTag2 Path

Two current behaviors undermine MegaTag2:

1. `Vision.isBetter(...)` adds X, Y, and heading standard deviations. MegaTag2
   deliberately assigns infinite heading uncertainty, so it can never beat a
   simultaneous finite-heading MegaTag1 observation even when its translation
   is better. Adding meters and radians into one score is also dimensionally
   invalid.
2. `RobotContainer` constructs Limelight IO with `drive::getHeading`. That is
   the vision-corrected estimator heading. MegaTag2 should receive the raw gyro
   reference, otherwise vision influences the heading used to calculate the
   next vision result.

The second issue matches the “MegaTag / Gyro Feedback Loop of Doom” reported in
the
[4744 Open Alliance build thread](https://www.chiefdelphi.com/t/ninjas-4744-2026-build-thread-open-alliance/505741?page=4).

Change the Limelight configuration to choose an explicit operating strategy:

- **Normal enabled localization:** MegaTag2 translation with raw Pigeon/NavX
  yaw; heading standard deviation remains infinite.
- **Initialization or gyro recovery:** a carefully gated MegaTag1 solution, or
  a known autonomous starting heading.
- **Fallback:** MegaTag1 only when MegaTag2 is unavailable or unhealthy, never
  both estimates from the same image as independent measurements.

Limelight firmware must be part of the checked robot configuration. Limelight
OS 2026.1 fixed invalid/empty MegaTag2 outputs and IMU convergence errors, as
documented in the
[2026.1 critical-fixes announcement](https://www.chiefdelphi.com/t/limelight-2026-1-critical-fixes-and-updates/519044).

### P0: Remove Rolling Vision Smoothing and Camera Pre-Fusion

The current code aligns camera solutions to the newest camera timestamp,
inverse-variance fuses them, stores that fused result for 250 ms, aligns the
old fused results forward again, and fuses them a second time. This has four
problems:

- repeated frames are correlated, but inverse-variance fusion treats them as
  independent evidence;
- the calculated standard deviation shrinks approximately with the square root
  of the number of stored frames even when no new independent information was
  gained;
- the oldest evidence is submitted again in multiple later estimator updates;
  and
- smoothing and next-loop queue application add latency to auto-aim and path
  correction.

WPILib's estimator already rolls back to each measurement timestamp, applies
the correction, and replays odometry. Its 2027 API also exposes `sampleAt(...)`.
See the
[WPILib 2027 PoseEstimator API](https://github.wpilib.org/allwpilib/docs/2027/java/org/wpilib/math/estimator/PoseEstimator.html).

Replace the custom fusion path with:

1. build at most one chosen pose solution per physical camera frame;
2. validate each observation independently;
3. sort accepted observations by timestamp;
4. submit each once with its own standard deviations; and
5. let the authoritative estimator perform temporal correction.

If simultaneous cameras see the same tags, their errors are still correlated.
Do not manufacture extra confidence by pre-combining them. Conservative
per-camera standard deviations are safer than an assumed independence model.

If presentation smoothing is useful for a dashboard, smooth a display-only
pose. Controllers and the estimator should consume the unsmoothed fused state.

### P1: Make Estimator History the Single Pose History

Az-RBSI currently maintains both WPILib's internal estimator history and an
external concurrent pose buffer. The external buffer is populated after
odometry updates, but earlier entries are not rewritten when a delayed vision
correction changes the estimator. The two histories can therefore describe
different fused trajectories.

Use `SwerveDrivePoseEstimator.sampleAt(timestamp)` for estimator-relative
history. Retain separate raw wheel/gyro history only when another algorithm
specifically needs odometry that has not been corrected by vision. Name the two
frames explicitly, for example:

- `fieldToRobotEstimated`: fused field pose; and
- `odomToRobotRaw`: continuous local odometry pose.

This follows the common robotics model discussed in the WPILib pose-estimator
drift thread: raw odometry may remain in its own continuous frame while vision
updates the mapping from that frame to the field. See
[WPILib pose estimator drift discussion](https://www.chiefdelphi.com/t/psa-wpilib-pose-estimator-drifts-and-ignores-most-vision-measurements/496876).

### P1: Apply New Vision in the Same Robot Loop

`DriveOdometry` runs before `Vision`, so the measurement produced by `Vision`
is normally applied at the end of the next loop. The timestamp remains correct,
but downstream users see the correction roughly 20 ms later than necessary.

After removing custom time alignment, introduce a small `PoseFusion` virtual
subsystem that runs after all localization IO. It should acquire the estimator
lock, process accepted observations in chronological order under both a count
and time budget, and publish one coherent pose snapshot for later subsystems.
This preserves deterministic ownership without making camera IO block the
odometry sampling thread.

### P1: Make Odometry Samples Atomic and Fail Explicitly

`DriveOdometry` treats module 0's timestamp array as canonical. When another
module history is shorter, it repeats that module's last position. That keeps
the estimator running, but creates a synthetic wheel configuration that can
move or rotate the estimated robot incorrectly.

Replace parallel queues with atomic `OdometrySample` records where practical:

```java
record OdometrySample(
    double timestamp,
    Rotation2d gyroYaw,
    SwerveModulePosition[] modulePositions,
    int validityMask) {}
```

At minimum, process only the common valid prefix, log per-source mismatch and
age, and discard an incomplete sample rather than repeating a stale wheel.
Track effective sample frequency, jitter, maximum age, and consecutive invalid
samples.

The gyro-disconnected alert currently says kinematics is used as a fallback,
but the estimator continues receiving the last/current IMU value. Implement the
claimed behavior by integrating kinematic `dtheta` from module deltas while the
gyro is unavailable, with a prominent degraded-localization state. Re-anchor
heading only through an explicit recovery policy when the gyro returns.

### P1: Calibrate Uncertainty from Data, Not Only Heuristics

The current distance-squared/tag-count model is a good starting shape, but its
2 cm translation and 0.06 rad heading baselines are global constants, and the
estimator itself is constructed with WPILib's default state deviations.

Make both state and measurement noise explicit per robot. Use repeatable tests:

- stationary camera scatter at several distances and view angles;
- repeated measured translations and rotations;
- combined translation and rotation;
- acceleration, defense impacts, and bump traversal;
- one-tag versus multi-tag observations;
- partial occlusion and adverse lighting; and
- each camera independently.

Log raw vision pose, raw odometry pose, fused pose, timestamp/receipt age,
innovation, tag IDs, distance, ambiguity, standard deviations, robot speed,
angular rate, pitch/roll, and accept/reject reason. Fit conservative residual
envelopes offline, then replay candidate policies through AdvantageKit logs.

The 4744 build thread describes calibrating effective wheel radius until
odometry agreed with measured/vision displacement. That is useful, but use an
external tape, survey point, or overhead reference as the ground truth so a
camera extrinsic error is not baked into wheel radius. Camera transform and
wheel-radius calibration must be separate experiments.

### P1: Separate Global Localization from Final Alignment

The global fused pose should remain the source for trajectories, field-aware
decisions, and long-range approach. It should not be the only measurement used
for the last centimeters of a precision alignment.

Mechanical Advantage described global localization as broadly robust but
specialized tag-relative estimation as more responsive for final alignment.
See its
[global and specialized pose-estimation discussion](https://www.chiefdelphi.com/t/frc-6328-mechanical-advantage-2025-build-thread/477314/85).
Orbit 1360 reported the same two-layer architecture in its
[2026 Open Alliance thread](https://www.chiefdelphi.com/t/frc-1360-2026-build-thread/507908):
continuous approximate field pose plus on-command precise tag-relative pose.

Add a separate `AlignmentObservation` output containing target-relative
translation/yaw, timestamp, quality, and source. A final-alignment controller
may use it directly while retaining global pose as a fallback. Do not feed a
specialized local correction into the global estimator unless it satisfies the
global observation contract.

### P1: Reconcile Disabled Behavior and Documentation

The documentation says disabled vision uses repeated fixed-alpha pose blending
and estimator resets. The implementation now performs one initialization reset
and then calls normal `addVisionMeasurement(...)`; calculated alpha values are
not applied. This mismatch makes tuning constants and logged labels misleading.

Adopt a small explicit state machine:

- `UNINITIALIZED`: collect a short, mutually consistent multi-frame consensus;
- `INITIALIZED_DISABLED`: reset once, then accept normal gated observations or
  freeze according to a documented policy;
- `ENABLED`: normal estimator fusion; and
- `DEGRADED`: no valid absolute source or gyro failure.

Delete unused blend constants and update `RBSI-PoseBuffer.md` after the chosen
policy is implemented.

## Vendordep and Tooling Recommendations

### WPILib `SwerveDrivePoseEstimator`: Canonical Fusion Layer

WPILib is not a vendordep, but it should remain the portable canonical
estimator. It already provides timestamped latency compensation, per-measurement
uncertainty, reset APIs, and estimator history. Do not fork it to replace
standard deviations with arbitrary percentage weights; that makes upgrades and
cross-team support harder.

### PhotonLib: Default Camera Integration

PhotonLib is already installed. Increase its role rather than duplicating pose
solving in RBSI:

- use `PhotonPoseEstimator` for the active field layout and robot-to-camera
  transform;
- prefer coprocessor multi-tag PNP for general localization;
- evaluate `CONSTRAINED_SOLVEPNP` when a timestamped raw heading is available;
- keep the solver strategy explicit per camera; and
- retain RBSI's logged `LocalizationObservation` as the replay boundary.

PhotonVision 2027 exposes constrained solve PNP as a supported strategy, with
heading data supplied for each frame. See the
[PhotonPoseEstimator strategy API](https://javadocs.photonvision.org/release/org/photonvision/PhotonPoseEstimator.PoseStrategy.html).
Team 4533's 2026 off-RIO constrained-solve work also illustrates the benefit of
coprocessor solving and graceful fallback, while emphasizing that incorrect
camera extrinsics or gyro interpolation defeat sophisticated math. See
[Whacknet](https://www.chiefdelphi.com/t/4533-phoenix-whacknet-off-rio-constrained-solve-for-apriltags-zero-allocation-udp-vision/518777).

### Phoenix 6 Generated Swerve: Preferred All-CTRE Backend Candidate

Phoenix 6 is already installed. Its generated `SwerveDrivetrain` owns a
high-rate odometry thread, exposes raw and fused headings, accepts timestamped
vision measurements, and reports update-period telemetry. For a Phoenix-only
robot, an adapter around that API could replace much of RBSI's custom Phoenix
sampling code and eliminate divergent implementations.

Do this only behind a common `DriveLocalizationBackend` interface and verify
AdvantageKit replay requirements first. Keep RBSI's portable WPILib backend for
mixed hardware. Never run the Phoenix estimator and the RBSI estimator as two
competing authorities.

### YAGSL: Optional Mixed-Hardware Backend, Not an Added Layer

YAGSL is an official vendordep option and already wraps
`SwerveDrivePoseEstimator` plus `addVisionMeasurement(...)`. Its stated purpose
is standardized, vendor-neutral swerve configuration. See
[YAGSL vision odometry](https://docs.yagsl.com/overview/our-features/vision-odometry).

Az-RBSI currently consumes YAGSL-style configuration while retaining its own
drive and estimator implementation. Teams that want YAGSL should be offered a
complete YAGSL backend; do not mix YAGSL's estimator with RBSI's estimator.
Because 2026 YAGSL availability depended on MapleSim and other vendor updates,
pin and validate the exact 2027 version before making it a default.

### QuestNavLib: Experimental Independent Localization Source

QuestNavLib is a vendordep for consuming timestamped VSLAM pose frames from a
Meta Quest. Its 2026 API exposes all unread frames, timestamps, connectivity,
and tracking state. See the
[QuestNav robot-code guide](https://questnav.gg/docs/getting-started/robot-code/).

Open Alliance Team 8324 reported combining QuestNav with PhotonVision and
MapleSim, specifically valuing pose continuity over the REBUILT bump. Other
2026 users reported promising autos but packaging and initial-pose challenges.
See
[8324's build thread](https://www.chiefdelphi.com/t/frc-8324-meco-robotics-2026-open-alliance-build-thread/510654)
and the
[QuestNav 2026 discussion](https://www.chiefdelphi.com/t/questnav-2026/514607).

Implement QuestNav as another `LocalizationIO` source with conservative
uncertainty, explicit tracking-loss rejection, AprilTag-based initialization,
and wheel/gyro/AprilTag fallbacks. It should first run in shadow mode: log its
pose and residuals without controlling the robot. Promote it only after event-
length vibration, thermal, power, tracking-loss, and recovery tests.

### MapleSim: Validation Tool, Not a Localization Source

MapleSim is available through the official vendordep catalog as an FRC Java
physics engine. It can improve drivetrain collision, slip, bump, and camera
simulation, which makes it useful for regression tests of estimator failure
modes. It does not provide field truth on a real robot and should not become a
second pose authority.

Evaluate it only if its drivetrain simulation is materially better than
`DriveSimPhysics` for the tests listed below. Retain AdvantageKit replay as the
primary method for validating behavior against real sensor streams.

### AdvantageKit and AdvantageScope: Keep and Expand

Both already fit RBSI's goals. Add derived residual and health channels, plus an
offline replay harness that can compare candidate policies against the same
log. A useful replay report should include accepted measurements, false accepts,
false rejects, maximum correction, correction latency, pose discontinuity,
loop time, and queue drops.

## Proposed Target Architecture

```text
Drive sensor backend
  Phoenix generated | RBSI portable | complete YAGSL backend
                     |
              raw timestamped odometry
                     |
       one authoritative pose estimator
                     |
       +-------------+------------------+
       |                                |
global estimated pose/history       raw odom pose/history
       ^
       |
PoseFusion: chronological, bounded, same-loop application
       ^
       |
LocalizationObservation validator
       ^
       |
PhotonLib | Limelight | QuestNav | replay fixtures

Separate path:
AlignmentObservation -> final-alignment controller -> global-pose fallback
```

Recommended observation contract:

```java
record LocalizationObservation(
    String source,
    long frameId,
    double measurementTimestamp,
    double receiptTimestamp,
    Pose2d fieldToRobot,
    Matrix<N3, N1> standardDeviations,
    LocalizationStrategy strategy,
    int[] tagIds,
    double averageDistanceMeters,
    double ambiguity,
    boolean sourceHealthy) {}
```

The contract should be immutable and validated once. The estimator should never
need to know whether a pose came from PhotonVision, Limelight, or VSLAM.

## Implementation Sequence

### Phase 1: Correctness and Observability

- Add finite, timestamp-coverage, age, future-skew, and innovation gates.
- Log source, strategy, frame ID, age, residual, uncertainty, and rejection
  reason.
- Feed Limelight raw gyro yaw and explicitly select MT1 or MT2.
- Add tests demonstrating that NaN, infinity, stale/future timestamps, and
  post-reset frames never reach the estimator.

### Phase 2: Simplification

- Submit each accepted observation once and remove rolling smoothing.
- Replace custom alignment history with estimator `sampleAt(...)`.
- Add same-loop `PoseFusion` ordering.
- Remove dead disabled-blend code and reconcile documentation.

### Phase 3: Odometry Robustness

- Introduce atomic odometry samples or strict common-prefix validation.
- Implement and test the gyro-disconnected kinematic fallback.
- Make estimator state deviations configurable.
- Add wheel-radius, track-width, camera-extrinsic, latency, and slip test
  procedures.

### Phase 4: Standardized Backends

- Adopt `PhotonPoseEstimator` strategies inside the Photon IO adapter.
- Prototype a Phoenix generated-swerve backend for all-CTRE robots.
- Decide whether a complete YAGSL backend is worth its maintenance cost.
- Add QuestNav in shadow mode; evaluate MapleSim for failure-mode simulation.

## Acceptance Tests

The revised toolchain should pass the following before becoming the template
default:

| Test | Required outcome |
| --- | --- |
| Stationary 10-minute run | No pose walk outside the measured uncertainty envelope; no NaNs. |
| Measured 5 m out-and-back | Bounded raw-odometry scale error and repeatable fused return error. |
| Translation while rotating | No unexplained lateral drift after geometry and wheel-radius calibration. |
| Single-camera disconnect | Pose remains continuous; health changes promptly; no queue backlog. |
| All-camera disconnect | Odometry continues; automation receives an explicit degraded state. |
| Bad frame injection | NaN, field-center zero, stale, future, and large-jump poses are rejected. |
| Gyro disconnect/reconnect | Kinematic fallback remains bounded and recovery does not jump heading. |
| Bump/impact traversal | Slip is visible in logs and vision recovery remains bounded. |
| Pose reset with queued frames | Every pre-reset observation is rejected. |
| Multi-camera disagreement | Each source is logged separately; one bad camera cannot dominate. |
| Replay regression | Reprocessing the same log yields deterministic accept/reject decisions. |
| Loop-load test | Odometry sampling stays healthy and pose-fusion work remains within budget. |

## Decision Summary

Az-RBSI does not need a more elaborate estimator. It needs fewer overlapping
estimation layers, stricter data contracts, and better empirical calibration.
The custom high-rate sensor plumbing is valuable, especially for portable
mixed-vendor support. The custom vision fusion and smoothing are the part to
retire.

Standardize the interface at the observation and backend boundaries, let one
well-tested estimator own latency compensation, and use vendordeps to replace
vendor-specific plumbing only when they can be made the sole owner of that
layer. That approach improves precision, makes failure behavior auditable, and
reduces the amount of localization code every adopting team must understand.
