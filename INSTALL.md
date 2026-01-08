# Az-RBSI Installation Instructions

The Az-RBSI is available as a [Template Repository](
https://docs.github.com/en/repositories/creating-and-managing-repositories/creating-a-repository-from-a-template#creating-a-repository-from-a-template)
for teams to use for creating their own 2026 FRC robot code.  These instructions
assume that [you](
https://docs.github.com/en/get-started/start-your-journey/creating-an-account-on-github)
and/or [your team](
https://docs.github.com/en/get-started/learning-about-github/types-of-github-accounts#organization-accounts)
already have a GitHub account where you will store your 2026 FRC robot code.

--------

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
https://github.com/wpilibsuite/2026Beta)
(Driver Station and associated tools), and the [2026 CTRE Phoenix Tuner X](
https://v6.docs.ctr-electronics.com/en/stable/docs/tuner/index.html).  Take a
moment to update all software and firmware before attempting to load your new
robot project.

Please note that you need these _minimum_ versions of the following components:

* WPILib ` v2026.1.1-beta-1`
* RoboRIO image `FRC_roboRIO_2026_v1.2`

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


--------

### Updating your project based on the latest released version of Az-RBSI

As the season progresses, the Az-RBSI developers may add additional features
to the codebase based on user feedback and developing understanding of needed
functionality to compete well in the 2026 REBUILT game.

The Az-RBSI includes a GitHub Action that will cause your robot project
repository on GitHub to check for new updates to the template on a weekly
basis.  If a new version has been released, the `github-actions` bot will
automatically create a [Pull Request](
https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/proposing-changes-to-your-work-with-pull-requests/about-pull-requests)
in your repository that includes all of the changes since either you created
the 2026 robot code or the last time you updated.  All you need to do to
accept the changes is to merge the pull request (assuming no conflicts).

If you wish to check for updates more frequently, you may force the "Sync with
Az-RBSI Template" process to run under the "Actions" tab on your repository's
GitHub page.

The update process has been re-engineered for 2026, and *should* be a straight
list of the commits that have been applied to the Az-RBSI template since the
cloning or last update.  This process *should* remove files that have been
renamed (*e.g.*, `vendordeps` files that are labeled as "beta" in the months
prior to the start of the season), but it is important to inspect the list of
file changes.  Please submit a [GitHub Issue](https://github.com/AZ-First/Az-RBSI/issues)
if you have problems with the update process.
