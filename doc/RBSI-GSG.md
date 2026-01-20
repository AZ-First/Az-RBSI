# Az-RBSI Getting Started Guide

This page includes detailed steps for getting your new Az-RBSI-based robot code
up and running for the 2026 REBUILT season.

--------

### Before you deploy to your robot

Before you deploy code to your robot, there are several modifications you need
to make to the code base.

All of the code you will be writing for your robot's subsystems and
modifications to extant RBSI code will be done to files within the
`src/main/java/frc/robot` directory (and its subdirectories).

1. **Controller Type**: The Az-RBSI expects an Xbox-style controller -- if you
   have a PS4 or other, substitute the proper command-based controller class
   for `CommandXboxController` near the top of the `RobotContainer.java` file.

2. **Robot Project Constants**: All of the configurable values for your robot
   will be in the ``Constants.java`` file.  This file contains the outer
   ``Constants`` class with various high-level configuration variables such as
   ``swerveType``, ``autoType``, ``visionType``, and whether your team has
   purchased a [CTRE Pro license](https://v6.docs.ctr-electronics.com/en/stable/docs/licensing/team-licensing.html)
   for unlocking some of the more advanced communication and control features
   available for CTRE devices.

3. **Robot Physical Constants**: The next four classes in ``Constants.java``
   contain information about the robot's physical characteristics, power
   distribution information, all of the devices (motors, servos, switches)
   connected to your robot, and operator control preferences.  Work through
   these sections carefully and make sure all of the variables in these classes
   match what is on your robot *before* deploying code.  Power monitoring in
   Az-RBSI matches subsystems to ports on your Power Distribution Module, so
   carefully edit the `RobotDevices` class of `Constants.java` to include the
   proper power ports for each motor in your drivetrain, and include any motors
   from additional subsystems you add to your robot.

--------

### Tuning constants for optimal performance

4. Over the course of your robot project, you will need to tune PID parameters
   for both your drivebase and any mechanisms you build to play the game.
   AdvantageKit includes detailed instructions for how to tune the various
   portions of your drivetrain, and we **STRONGLY RECOMMEND** you work through
   these steps **BEFORE** running your robot.

   * [Tuning for drivebase with CTRE components](https://docs.advantagekit.org/getting-started/template-projects/talonfx-swerve-template#tuning)
   * [Tuning for drivebase with REV components](https://docs.advantagekit.org/getting-started/template-projects/spark-swerve-template/#tuning)

   Similar tuning can be done with subsystem components (flywheel, intake, etc.).

5. Power monitoring by subsystem is included in the Az-RBSI.  In order to
   properly match subsystems to ports on your Power Distribution Module,
   carefully edit the `RobotDevices` of `Constants.java` to include the
   proper power ports for each motor in your drivetrain, and include any
   motors from additional subsystems you add to your robot.  To include
   additional subsystems in the monitoring, add them to the [`m_power`
   instantiation](
   https://github.com/AZ-First/Az-RBSI/blob/38f6391cb70c4caa90502710f591682815064677/src/main/java/frc/robot/RobotContainer.java#L154-L157) in the `RobotContainer.java` file.

6. In the `Constants.java` file, the classes following `RobotDevices` contain
   individual containers for robot subsystems and interaction methods.  The
   `OperatorConstants` class determines how the OPERATOR interacts with the
   robot.  `DriveBaseConstants` and `FlywheelConstants` (and additional classes
   you add for your own mechanisms) contain human-scale conversions and limits
   for the subsystem (_e.g._, maximum speed, gear ratios, PID constants, etc.).
   `AutoConstants` contains the values needed for your autonomous period method
   of choice (currently supported are MANUAL -- you write your own code;
   PATHPLANNER, and CHOREO).  The next two are related to robot vision, where
   the vision system constants are contained in `VisionConstants`, and the
   physical properties (location, FOV, etc.) of the cameras are in `Cameras`.

--------

### Robot Development

As you program your robot for the 2026 (REBUILT) game, you will likely be
adding new subsystems and mechanisms to control and the commands to go with
them.  Add new subsystems in the `subsystems` directory within
`src/main/java/frc/robot` -- you will find an example flywheel already included
for inspiration.  New command modules should go into the `commands` directory.

The Az-RBSI is pre-plumbed to work with both the [PathPlanner](
https://pathplanner.dev/home.html) and [Choreo](
https://sleipnirgroup.github.io/Choreo/) autonomous path planning software
packages -- select which you are using in the `Constants.java` file.
Additionally, both [PhotonVision](https://docs.photonvision.org/en/latest/) and
[Limelight](
https://docs.limelightvision.io/docs/docs-limelight/getting-started/summary)
computer vision systems are supported in the present release.

--------

### Included 3D Prints

To help teams with standardized enclosures for their PhotonVision Orange Pi's
and CTRE CANivores, we include three 3D print files as part of the "Assets"
section of [each release](https://github.com/AZ-First/Az-RBSI/releases).

* The [CANivore cable holder](https://github.com/AZ-First/Az-RBSI/releases/download/v26.0.0-rc2/Canivore.Cable.holder.STL)
  is designed to hold a [right-angle USB-C cable](https://www.amazon.com/dp/B092ZS6SJG)
  onto the CANivore in a way that won't get knocked loose if your robot hits
  something.  The entire assembly can be attached to the RoboRIO using
  [double-sided mounting tape](https://www.amazon.com/dp/B00FUEN2GK).

* The [Orange Pi Double Case](https://github.com/AZ-First/Az-RBSI/releases/download/v26.0.0-rc2/Orange.Pi.Double.case.STL)
  and [Lid](https://github.com/AZ-First/Az-RBSI/releases/download/v26.0.0-rc2/Orange.Pi.Double.case.lid.STL)
  are designed to hold one or two [Orange Pi 5](https://www.amazon.com/dp/B0BN17PWWB)'s
  (not **B** or **Pro** or **Max**) (and connect up to 4 cameras).  If only using one
  Orange Pi, mount it in the "upper" position for airflow.  Also requires:

   * 2x [128 GB micro SD card](https://www.amazon.com/dp/B0B7NTY2S6)
   * 4x [M2.5x6mm+6mm Male-Female Hex Standoff](https://www.amazon.com/gp/product/B08F2F96HM) (under the bottom Pi)
   * 4x [M2.5x25mm+6mm Male-Female Hex Standoff](https://www.amazon.com/gp/product/B08F2DBNZW) (between the two Pi's)
   * 4x [M2.5x20mm Female-Female Hex Standoff](https://www.amazon.com/gp/product/B08F2HZN4R) (atop the upper Pi)
   * 8x [M2.5x8mm Machine Screws](https://www.amazon.com/gp/product/B07MLB1627) (through case and into standoffs)
   * 2x [Cooling Fan 40mm 5V DC + Grill](https://www.amazon.com/gp/product/B08R1CXGCJ) (attaches to side of case)
   * 1x [Redux Robotics Zinc-V Regulator](https://shop.reduxrobotics.com/products/zinc-v) OR [Pololu 5V, 5.5A Step-Down Voltage Regulator](https://www.pololu.com/product/4091) OR [Pololu 5V, 3A Step-Up/Step-Down Voltage Regulator](https://www.pololu.com/product/4082) (Power regulation for the Pi's)
   * 2x [90-Degree USB-C to USB-C Cable, 10 inch](https://www.amazon.com/dp/B0CG1PZMVG) for ZINC-V or [90-Degree USB-C to 2 Pin Bare Wire, 10 inch](https://www.amazon.com/gp/product/B0CY2J5H3K) for Pololu (for powering Pi's)
   * 3x [M2x8mm Machine Screws](https://www.amazon.com/gp/product/B07M6RTWCC) (for attaching the Pololu to the lid)

   **NOTE: Powering the case with a Pololu requires soldering the USB-C cables
   and the 18/2 AWG wires to the Pololu.**  This requires patience.  Using the
   Zinc-V requires no soldering, but introduces an extra USB-C connection.

   See the [PhotonVision Wiring documentation
   ](https://docs.photonvision.org/en/latest/docs/quick-start/wiring.html) for
   more details.

   Mounting the case to the robot requires 4x #10-32 nylock nuts (placed in the
   hex-shaped mounts inside the case) and 4x #10-32 bolts.
