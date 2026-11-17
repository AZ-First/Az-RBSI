# 2027 alpha-8 configuration status

This page describes the checked-in setup. A successful build or simulation does not certify a
physical robot. Confirm CAN IDs, bus routing, geometry, camera transforms, and mechanism limits on
your own hardware before driving.

| Area | `alpha8` (Commands V2) | `commands_v3` (Commands V3) |
| --- | --- | --- |
| Robot and controller | `COMPBOT` on Systemcore; desktop runs replay | Same |
| Drive | Phoenix 6 and Pigeon, configured path | Same |
| YAGSL | Runtime deferred; selecting it throws | V3 vendordep builds, but runtime remains deferred |
| Autonomous | `MANUAL` selected; PathPlanner and Choreo integrations compile but need paths and robot tuning | `MANUAL` selected; PathPlanner and Choreo disabled pending V3 libraries |
| Vision | PhotonVision selected with a temporary 2027 field-layout compatibility shim | Same |
| Example flywheel | Disabled on real hardware by default; simulation model available | Same |

The selectors live in `Constants.java`. The default `SwerveType.PHOENIX6` is the only wired 2027
drive path. The deploy YAGSL JSON uses an older format and a roboRIO `navx_spi` gyro. Regenerate it
for 2027 and select a Systemcore-compatible gyro before restoring the YAGSL mapping. On V3 the
vendordep alone does not enable that mapping. See `Commands-V3-Branch.md` on the V3 branch.

The V2 PathPlanner and Choreo code is an integration example. Supply valid paths/trajectories,
check the field and pose conventions, and tune the robot before selecting those modes. On V3,
keep `AutoType.MANUAL` until V3-compatible libraries and adapters are available. The PhotonVision
shim is tracked in `PHOTONVISION-COMPAT.md` and should be removed after a compatible release.

## Example flywheel

`Constants.kExampleFlywheelHardware` defaults to `DISABLED`. On a real Systemcore, this creates
no flywheel motor controllers and registers neither the right-bumper flywheel binding nor flywheel
SysId choices. `FLYWHEEL_LEADER` and `FLYWHEEL_FOLLOWER` are sample CAN and power-port numbers,
not an installed mechanism. In simulation, `FlywheelIOSim` and its controls remain available.
Replay uses no hardware IO or flywheel controls.

To install a physical example, first replace those device IDs and ports, verify the gearing,
motor direction, current limits and SysId limits, then select `TALON_FX` or `SPARK_MAX` in
`Constants.kExampleFlywheelHardware`. This opts the real robot into the matching IO, bumper
binding, and flywheel SysId choices (PathPlanner chooser on V2, manual chooser on V3). See
`RBSI-SysId.md` before characterization. For a competition mechanism, replace the example with
its own IO, commands, controls, and power monitoring configuration.
