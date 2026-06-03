# Az-RBSI Documentation

This directory contains the project-level documentation for the Az-RBSI robot
code template.

## Start Here

- [INSTALL.md](INSTALL.md): creating a project from the template and preparing
  the development environment.
- [RBSI-GSG.md](RBSI-GSG.md): first robot-code changes after the project is
  created.
- [RBSI-Constants.md](RBSI-Constants.md): how `Constants.java` is organized and
  which values teams should tune first.

## Robot Bring-Up And Tuning

- [RBSI-Drive.md](RBSI-Drive.md): drivetrain setup, odometry, characterization,
  and drive tuning.
- [RBSI-SysId.md](RBSI-SysId.md): SysId routines for the example flywheel and
  how to use the generated data.
- [RBSI-PoseBuffer.md](RBSI-PoseBuffer.md): detailed design notes for
  time-aligned odometry and vision fusion.

## Vision And Autonomous

- [RBSI-Vision.md](RBSI-Vision.md): PhotonVision/Limelight setup, camera
  transforms, filtering, simulation, and troubleshooting.
- [RBSI-Autonomous.md](RBSI-Autonomous.md): selecting and tuning Manual,
  PathPlanner, Choreo, and Autopilot workflows.

## Suggested Reading Order For A New Team

1. `INSTALL.md`
2. `RBSI-GSG.md`
3. `RBSI-Constants.md`
4. `RBSI-Drive.md`
5. `RBSI-Vision.md`
6. `RBSI-Autonomous.md`
7. `RBSI-SysId.md`
8. `RBSI-PoseBuffer.md`

`RBSI-PoseBuffer.md` is more of a design reference than a bring-up guide. Read
it when you need to understand why odometry and vision are ordered the way they
are.
