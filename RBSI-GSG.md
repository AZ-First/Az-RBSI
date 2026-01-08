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

4. HHHH




5. Power monitoring by subsystem is included in the Az-RBSI.  In order to
   properly match subsystems to ports on your Power Distribution Module,
   carefully edit the `RobotDevices` of `Constants.java` to include the
   proper power ports for each motor in your drivetrain, and include any
   motors from additional subsystems you add to your robot.  To include
   additional subsystems in the monitoring, add them to the [`m_power`
   instantiation](
   https://github.com/AZ-First/Az-RBSI/blob/38f6391cb70c4caa90502710f591682815064677/src/main/java/frc/robot/RobotContainer.java#L154-L157) in the `RobotContainer.java` file.

6. All of the constants for needed for tuning your robot should be in the
   `Constants.java` file in the `src/main/java/frc/robot` directory.  This file
   should be thoroughly edited to match the particulars of your robot.  Be sure
   to work through each section of this file and include the proper values for
   your robot.


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
