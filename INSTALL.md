# Az-RBSI Installation Instructions

The Az-RBSI is available as a [Template Repository](
https://docs.github.com/en/repositories/creating-and-managing-repositories/creating-a-repository-from-a-template#creating-a-repository-from-a-template)
for teams to use for creating their own 2026 FRC robot code.  These instructions
assume that [you](
https://docs.github.com/en/get-started/start-your-journey/creating-an-account-on-github)
and/or [your team](
https://docs.github.com/en/get-started/learning-about-github/types-of-github-accounts#organization-accounts)
already have a GitHub account where you will store your 2026 FRC robot code.

### Creating a 2026 FRC project from the Az-RBSI Template

From the [Az-RBSI GiuHub page](https://github.com/AZ-First/Az-RBSI/), click the "Use this template" button in the upper right corner of the page.

In the page that opens, select the Owner (most likely your team's account) and
Repository name (*e.g.*, "FRC-2026" or "REBUILT Robot Code" or whatever your team's naming convention
is) into which the create the new robot project.  Optionally, include a
description of the repository for your reference.  Select "public" or "private"
repository based on the usual practices of your team.

The latest release of Az-RBSI is in the `main` (default) branch, so it is
recommended to **not** select the "Include all branches" checkbox.

--------

### Software Requirements

The Az-RBSI requires the [2026 WPILib Installer](
https://github.com/wpilibsuite/allwpilib/releases) (VSCode and associated
tools), 2026 firmware installed on all hardware (motors, encoders, power
distribution, etc.), the [2026 NI FRC Game Tools](
https://www.ni.com/en/support/downloads/drivers/download.frc-game-tools.html)
(Driver Station and associated tools), and the [2026 CTRE Phoenix Tuner X](
https://v6.docs.ctr-electronics.com/en/stable/docs/tuner/index.html).  Take a
moment to update all software and firmware before attempting to load your new
robot project.

Please note that you need these _minimum_ versions of the following components:

* WPILib ` v2026.1.1-beta-1`
* RoboRIO image `FRC_roboRIO_2026_v2.0`

--------

### Setting up your new project

When your new robot code respository is created, it will have a single commit
that contains the entire Az-RBSI template for the current release.  (See the
[Az-RBSI Releases page](https://github.com/AZ-First/Az-RBSI/releases) for more
information about the latest release.)

Before you can start to use your code on your robot, there are several set up
steps you need to complete:

1. Add your team number to the `.wpilib/wpilib_preferences.json` file.  The
   generic Az-RBSI template contains a team number "0", and your code will not
   deploy properly if this variable is not set (*i.e.*, since VSCode looks for
   the RoboRIO on IP address `10.TE.AM.2`, it will not find anything if it
   tries to contact `10.0.0.2`.)  If you forget to change this value, you will
   get an error message when deploying code to your robot like:

   ```
   Missing Target!
   =============================================
   Are you connected to the robot, and is it on?
   =============================================
   GradleRIO detected this build failed due to not being able to find "roborio"!
   Scroll up in this error log for more information.
   ```

2. If you have an all-CTRE swerve base (*i.e.*, 8x TalonFX-controlled motors,
   4x CANCoders, and 1x Pigeon2), use Phoenix Tuner X to create a swerve
   project.  Follow the instructions in the Phoenix documentation for the
   [Tuner X Swerve Project Generator](
   https://v6.docs.ctr-electronics.com/en/latest/docs/tuner/tuner-swerve/index.html).
   This will generate the correct offsets and inversions for your drive train.

3. On the final screen in Tuner X, choose "Generate only TunerConstants" and
   overwrite the file located at `src/main/java/frc/robot/generated/TunerConstants.java`.

4. In `TunerConstants.java`, comment out the [last import](
   https://github.com/CrossTheRoadElec/Phoenix6-Examples/blob/1db713d75b08a4315c9273cebf5b5e6a130ed3f7/java/SwerveWithPathPlanner/src/main/java/frc/robot/generated/TunerConstants.java#L18)
   and [last method](
   https://github.com/CrossTheRoadElec/Phoenix6-Examples/blob/1db713d75b08a4315c9273cebf5b5e6a130ed3f7/java/SwerveWithPathPlanner/src/main/java/frc/robot/generated/TunerConstants.java#L171-L175).
   Before removing them, both lines will be marked as errors in VSCode.

5. In `TunerConstants.java`, change `kSteerInertia` to `0.004` and
   `kDriveInertia` to `0.025` to allow the AdvantageKit simulation code to
   operate as expected.


**NOTE:** If you have any other combination of hardware (including REV NEOs,
NavX IMU, etc.) you will need to use the [YAGSL Swerve Configurator](
https://broncbotz3481.github.io/YAGSL-Example/) to configure the inputs for
your robot.  **Since the reference build recommends an all-CTRE swerve base**,
this functionality has not been extensively tested.  Any teams that adopt this
method are encouraged to submit bug reports and code fixes to the [Az-RBSI
repository](https://github.com/AZ-First/Az-RBSI).

6. The Az-RBSI expects an Xbox-style controller -- if you have a PS4 or other,
   substitute the proper command-based controller class for
   `CommandXboxController` near the top of the `RobotContainer.java` file in
   the `src/main/java/frc/robot` directory.

7. Power monitoring by subsystem is included in the Az-RBSI.  In order to
   properly match subsystems to ports on your Power Distribution Module,
   carefully edit the `CANandPowerPorts` of `Constants.java` to include the
   proper power ports for each motor in your drivetrain, and include any
   motors from additional subsystems you add to your robot.  To include
   additional subsystems in the monitoring, add them to the [`m_power`
   instantiation](
   https://github.com/AZ-First/Az-RBSI/blob/38f6391cb70c4caa90502710f591682815064677/src/main/java/frc/robot/RobotContainer.java#L154-L157) in the `RobotContainer.java` file.

8. All of the constants for needed for tuning your robot should be in the
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


--------

### Updating your project based on the latest released version of Az-RBSI

The Az-RBSI includes a GitHub Action that will cause your robot project
repository on GitHub to check for new versions of the template on a weekly
basis.  If a new version has been released, the `github-actions` bot will
automatically create a [Pull Request](
https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/proposing-changes-to-your-work-with-pull-requests/about-pull-requests)
in your repository that includes all of the changes.  All you need to do to
accept the changes is to merge the pull request (assuming no conflicts).

If you wish to check for updates more frequently, you may force the "Sync with
Az-RBSI Template" process to run under the "Actions" tab on your repository's
GitHub page.

Please note that this update process does NOT remove files that have been
renamed (*e.g.*, `vendordeps` files that are labeled as "beta" in the months
prior to the start of the season).  As such, it is important to inspect the
list of file changes and manually remove these kinds of files.  The Az-RBSI
developers will endeavor to list all such files on the [Az-RBSI Releases page](
https://github.com/AZ-First/Az-RBSI/releases), but *caveat emptor*.
