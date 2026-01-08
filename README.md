[![CI](https://github.com/AZ-First/Az-RBSI/actions/workflows/ci.yaml/badge.svg)](https://github.com/AZ-First/Az-RBSI/actions/workflows/ci.yaml)


![AzFIRST Logo](https://github.com/AZ-First/Az-RBSI/blob/main/AZ-First-logo.png?raw=true)

# Az-RBSI
Arizona's Reference Build and Software Implementation for FRC Robots (read: "A-Z-ribsy")


## Installation

Installation instructions are found in the [INSTALL.md](INSTALL.md) file, and the [Getting
Started Guide](RBSI-GSG.md) includes the steps you'll need to do before taking your robot
out for a spin.  See the [Releases Page](https://github.com/AZ-First/Az-RBSI/releases) for
details on the latest release, including restrictions and cautions.


## Purpose

The purpose of Az-RBSI is to help Arizona FRC teams with:
* Improving robot reliability / performance during “Autonomous Play”
* Improving robot build & endurance, gameplay reliability and troubleshooting
    skills
* Providing a standardized robot “stack” to allow for quick software setup and
    troubleshooting, and make it easier for Arizona teams to form effective
    in-state alliances


## Design Philosophy

The Az-RBSI is centered around a "Reference Build" robot that allows for teams
to communicate quickly and effectivly with each other about gameplay strategy
and troubleshooting.  Additionally, the consolidation around a standard robot
design allows for easier swapping of spare parts and programming modules.

The Az-RBSI software is an outline of an FRC robot program upon which teams can
build with their particular mechanisms and designs.  It weaves together popular
and currently maintained FIRST- and community-sponsored software libraries to
provide a baseline robot functionality that combines robust reliability with
effective logging for troubleshooting.


## Library Dependencies

* [WPILib](https://docs.wpilib.org/en/stable/index.html) -- FIRST basic libraries
* [AdvantageKit](
   https://github.com/Mechanical-Advantage/AdvantageKit/blob/main/docs/WHAT-IS-ADVANTAGEKIT.md)
   -- Logging
* [CTRE Phoenix6](
  https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/mechanisms/swerve/swerve-overview.html)
  / [YAGSL](https://yagsl.gitbook.io/yagsl) -- Swerve drive library
* [PathPlanner](https://pathplanner.dev/home.html) / [Choreo](
  https://sleipnirgroup.github.io/Choreo/) -- Autonomous path planning
* [PhotonVision](https://docs.photonvision.org/en/latest/) / [Limelight](
  https://docs.limelightvision.io/docs/docs-limelight/getting-started/summary)
  -- Robot vision / tracking
* [Autopilot](https://therekrab.github.io/autopilot/index.html) -- Drive-to-Pose semi-autonomous movements

## Further Reading

For tips on command-based programming, see this post:
https://bovlb.github.io/frc-tips/commands/best-practices.html
