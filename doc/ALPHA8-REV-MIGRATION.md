# REVLib 2027 migration

REVLib 2027 removed SPARK encoder conversion factors and the configurable
position wrapping range. `ModuleIOSpark`, `ModuleIOSparkCANcoder`, and
`ModuleIOBlended` now use raw encoder rotations/RPM at the controller boundary.
`SparkSwerveUnits` converts readings back to radians and radians/second for
`ModuleIO`, converts closed-loop requests to rotations/RPM, and scales the
SPARK gains to keep the previous physical units. Drive and steering odometry
queues use the same conversions as periodic telemetry. The CANcoder queue in
`ModuleIOSparkCANcoder` is decoded as rotations, matching its periodic data.

The build and unit tests pass. Before operating a physical Spark module,
verify wheel direction, steering shortest-path behavior across zero, odometry,
velocity control, and SysId. The 2027 firmware's filtering behavior also
changed, so steering and drive gains may need tuning after those checks.

REV's release notes: https://github.com/wpilibsuite/SystemCoreTesting/blob/main/REV.md
