# Az-RBSI Documentation

This directory contains the project-level documentation for the Az-RBSI robot
code template.

Use this page as the documentation map. The root [README](../README.md) is the
project front door; this page is the working table of contents.

## Table Of Contents

| Page | Use It For |
| --- | --- |
| [INSTALL.md](INSTALL.md) | Creating a project, setting team number, generating Phoenix Tuner X constants, and first setup tasks. |
| [RBSI-GSG.md](RBSI-GSG.md) | First robot-code changes after installation. |
| [RBSI-Constants.md](RBSI-Constants.md) | Understanding `Constants.java` and the tuning order for a new robot. |
| [RBSI-Drive.md](RBSI-Drive.md) | Phoenix/YAGSL drivetrain setup, odometry, drive tuning, and characterization. |
| [RBSI-Vision.md](RBSI-Vision.md) | PhotonVision/Limelight setup, camera transforms, filtering, simulation, and troubleshooting. |
| [RBSI-Autonomous.md](RBSI-Autonomous.md) | Manual autos, PathPlanner, Choreo, Autopilot, match lifecycle, and pose reset rules. |
| [RBSI-SysId.md](RBSI-SysId.md) | Running SysId routines and applying flywheel characterization results. |
| [RBSI-PoseBuffer.md](RBSI-PoseBuffer.md) | Design reference for time-aligned odometry and vision fusion. |
| [RBSI-AdvantageScope.md](RBSI-AdvantageScope.md) | Standard AdvantageScope layout tabs and how to add mechanism power/timing plots. |

## New User Bring-Up Path

1. [Install the project](INSTALL.md).
2. [Make first code changes](RBSI-GSG.md).
3. [Select robot, swerve, vision, and auto modes](RBSI-Constants.md).
4. [Generate or configure drivetrain constants](RBSI-Drive.md).
5. [Verify odometry before vision](RBSI-Drive.md#bring-up-checklist).
6. [Configure cameras and vision filtering](RBSI-Vision.md).
7. [Bring up a short autonomous path](RBSI-Autonomous.md).
8. [Open the standard AdvantageScope layout](RBSI-AdvantageScope.md).
9. [Run SysId for mechanisms and update gains](RBSI-SysId.md).

## Experienced User Navigation

- Phoenix Tuner X copy/rename rules:
  [INSTALL.md](INSTALL.md) and
  [`src/main/java/frc/robot/generated/README`](../src/main/java/frc/robot/generated/README)
- Constants tuning order:
  [RBSI-Constants.md](RBSI-Constants.md#recommended-tuning-order-for-a-new-robot)
- Drive/odometry symptoms:
  [RBSI-Drive.md](RBSI-Drive.md#troubleshooting)
- Vision pose jumps:
  [RBSI-Vision.md](RBSI-Vision.md#troubleshooting)
- PathPlanner/Choreo startup and reset behavior:
  [RBSI-Autonomous.md](RBSI-Autonomous.md#match-execution-flow)
- Pose buffer internals:
  [RBSI-PoseBuffer.md](RBSI-PoseBuffer.md)
- AdvantageScope pit-debug layout:
  [RBSI-AdvantageScope.md](RBSI-AdvantageScope.md)

## Recommended Reading Order

For a new team:

1. `INSTALL.md`
2. `RBSI-GSG.md`
3. `RBSI-Constants.md`
4. `RBSI-Drive.md`
5. `RBSI-Vision.md`
6. `RBSI-Autonomous.md`
7. `RBSI-SysId.md`
8. `RBSI-AdvantageScope.md`
9. `RBSI-PoseBuffer.md`

For troubleshooting during the season, start with the symptom-specific page,
then follow links back to constants and bring-up checklists.
