[![CI](https://github.com/AZ-First/Az-RBSI/actions/workflows/ci.yaml/badge.svg)](https://github.com/AZ-First/Az-RBSI/actions/workflows/ci.yaml)

![AzFIRST Logo](https://github.com/AZ-First/Az-RBSI/blob/main/AZ-First-logo.png?raw=true)

# Az-RBSI

Arizona's Reference Build and Software Implementation for FRC robots, read as
"A-Z-ribsy".

Az-RBSI is a robot-code template for teams that want a reliable swerve,
odometry, vision, autonomous, logging, and tuning baseline without starting from
an empty project.

## Documentation

Start with the full documentation index:

- [doc/README.md](doc/README.md)

Quick links:

- [Install and project setup](doc/INSTALL.md)
- [Getting started guide](doc/RBSI-GSG.md)
- [Constants guide](doc/RBSI-Constants.md)
- [Drive subsystem and Phoenix/YAGSL setup](doc/RBSI-Drive.md)
- [Vision setup and troubleshooting](doc/RBSI-Vision.md)
- [Autonomous setup](doc/RBSI-Autonomous.md)
- [SysId guide](doc/RBSI-SysId.md)
- [Pose buffer design notes](doc/RBSI-PoseBuffer.md)
- [AdvantageScope layout guide](doc/RBSI-AdvantageScope.md)

## New Team Flow

1. Read [INSTALL.md](doc/INSTALL.md) and create your project.
2. Follow [RBSI-GSG.md](doc/RBSI-GSG.md) for first robot-code edits.
3. Configure `Constants.java` with [RBSI-Constants.md](doc/RBSI-Constants.md).
4. Configure the drivetrain with [RBSI-Drive.md](doc/RBSI-Drive.md).
5. Bring up cameras with [RBSI-Vision.md](doc/RBSI-Vision.md).
6. Add autonomous paths with [RBSI-Autonomous.md](doc/RBSI-Autonomous.md).
7. Open the standard layout with [RBSI-AdvantageScope.md](doc/RBSI-AdvantageScope.md).
8. Characterize mechanisms with [RBSI-SysId.md](doc/RBSI-SysId.md).

## Experienced User Shortcuts

- Phoenix Tuner X generated constants:
  [RBSI-Drive.md](doc/RBSI-Drive.md#phoenix-tuner-x-constants) and
  [`src/main/java/frc/robot/generated/README`](src/main/java/frc/robot/generated/README)
- PathPlanner and Choreo lifecycle:
  [RBSI-Autonomous.md](doc/RBSI-Autonomous.md#match-execution-flow)
- Disabled odometry and vision behavior:
  [RBSI-PoseBuffer.md](doc/RBSI-PoseBuffer.md)
- Camera transforms, filtering, and simulation:
  [RBSI-Vision.md](doc/RBSI-Vision.md)
- Constants tuning order:
  [RBSI-Constants.md](doc/RBSI-Constants.md#recommended-tuning-order-for-a-new-robot)
- AdvantageScope pit-debug layout:
  [RBSI-AdvantageScope.md](doc/RBSI-AdvantageScope.md)

## Purpose

Az-RBSI helps FRC teams with:

- improving autonomous reliability and performance,
- improving robot build, endurance, gameplay reliability, and troubleshooting,
- standardizing a robot stack so teams can set up software quickly,
- making it easier for Arizona teams to form effective in-state alliances.

## Design Philosophy

Az-RBSI is centered around a reference robot that helps teams communicate
quickly about gameplay strategy and troubleshooting. A shared robot design also
makes it easier to swap spare parts and programming modules.

The software is a robot-program outline that teams can extend for their own
mechanisms and game strategy. It combines actively maintained FIRST and
community libraries with AdvantageKit logging so teams can diagnose problems
from real match data.

## Library Dependencies

- [WPILib](https://docs.wpilib.org/en/stable/index.html): FIRST robot libraries
- [AdvantageKit](https://docs.advantagekit.org/getting-started/what-is-advantagekit/): logging
- [CTRE Phoenix 6](https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/mechanisms/swerve/swerve-overview.html): CTRE swerve
- [PathPlanner](https://pathplanner.dev/home.html): autonomous path planning
- [PhotonVision](https://docs.photonvision.org/en/latest/) and
  [Limelight](https://docs.limelightvision.io/docs/docs-limelight/getting-started/summary):
  robot vision
- [Autopilot](https://therekrab.github.io/autopilot/index.html): teleop drive-to-pose

## Workshop Slides

2026 REBUILT kickoff workshops:

- [AZ RBSI and AdvantageKit](https://docs.google.com/presentation/d/1KOfODbdGbk8L_G25i7iYnaahoKr_Tzg54LJYN4yax_4/edit?usp=sharing)
- [Know Where You Are: PhotonVision for Alignment and Odometry](https://docs.google.com/presentation/d/1JWYmwpZYA2zBuNIj9kKBUC_O-i0d1-SW_6qsVxgPdCA/edit?usp=sharing)

2025 Reefscape kickoff workshop:

- [AZ Liftoff RBSI](https://docs.google.com/presentation/d/1c8A5RlPeEvKcj9yC66Ffvh5Os6jWyZiACoSRjDDETUs/edit?usp=sharing)

## Further Reading

- [Command-based best practices](https://bovlb.github.io/frc-tips/commands/best-practices.html)
- [RBSI releases](https://github.com/AZ-First/Az-RBSI/releases)
