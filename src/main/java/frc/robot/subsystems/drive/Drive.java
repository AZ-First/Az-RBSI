// Copyright (c) 2024-2026 Az-FIRST
// http://github.com/AZ-First
// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the AdvantageKit-License.md file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.drive.SwerveConstants.*;

import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.subsystems.imu.Imu;
import frc.robot.util.ConcurrentTimeInterpolatableBuffer;
import frc.robot.util.LocalADStarAK;
import frc.robot.util.RBSIEnum.Mode;
import frc.robot.util.RBSIParsing;
import frc.robot.util.RBSISubsystem;
import frc.robot.util.TimedPose;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * Drive subsystem (RBSISubsystem)
 *
 * <p>The Drive subsystem controls the individual swerve Modules and owns the odometry of the robot.
 * The odometry is updated from both the swerve modules and (optionally) the vision subsystem.
 */
public class Drive extends RBSISubsystem {

  // Declare Hardware
  private final Imu imu;
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final SysIdRoutine sysId;

  // Buffers for necessary things
  private final ConcurrentTimeInterpolatableBuffer<Pose2d> poseBuffer =
      ConcurrentTimeInterpolatableBuffer.createBuffer(DrivebaseConstants.kHistorySize);
  private final ConcurrentTimeInterpolatableBuffer<Double> yawBuffer =
      ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(DrivebaseConstants.kHistorySize);
  private final ConcurrentTimeInterpolatableBuffer<Double> yawRateBuffer =
      ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(DrivebaseConstants.kHistorySize);

  // Declare an alert
  private final Alert gyroDisconnectedAlert =
      new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

  // Declare odometry and pose-related variables
  static final Lock odometryLock = new ReentrantLock();
  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private final SwerveModulePosition[] odomPositions = {
    new SwerveModulePosition(),
    new SwerveModulePosition(),
    new SwerveModulePosition(),
    new SwerveModulePosition()
  };
  private SwerveDrivePoseEstimator m_PoseEstimator =
      new SwerveDrivePoseEstimator(kinematics, Rotation2d.kZero, lastModulePositions, Pose2d.kZero);

  // Declare PID controller and siumulation physics
  private ProfiledPIDController angleController;
  private DriveSimPhysics simPhysics;

  // Pose reset gate (vision + anything latency-sensitive)
  private volatile long poseResetEpoch = 0; // monotonic counter
  private volatile double lastPoseResetTimestamp = Double.NEGATIVE_INFINITY;

  /** Constructor */
  public Drive(Imu imu) {
    this.imu = imu;

    // Define the Angle Controller
    angleController =
        new ProfiledPIDController(
            DrivebaseConstants.kPSPin,
            DrivebaseConstants.kISPin,
            DrivebaseConstants.kDSpin,
            new TrapezoidProfile.Constraints(
                getMaxAngularSpeedRadPerSec(), getMaxLinearAccelMetersPerSecPerSec()));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    // If REAL (i.e., NOT simulation), parse out the module types
    if (Constants.getMode() == Mode.REAL) {

      // Case out the swerve types because Az-RBSI supports a lot
      switch (Constants.getSwerveType()) {
        case PHOENIX6:
          // This one is easy because it's all CTRE all the time
          for (int i = 0; i < 4; i++) {
            modules[i] = new Module(new ModuleIOTalonFX(i), i);
          }
          break;

        case YAGSL:
          // Then parse the module(s)
          Byte modType = RBSIParsing.parseModuleType();
          for (int i = 0; i < 4; i++) {
            switch (modType) {
              case 0b00000000: // ALL-CTRE
                if (kImuType.equals("navx") || kImuType.equals("navx_spi")) {
                  modules[i] = new Module(new ModuleIOTalonFX(i), i);
                } else {
                  throw new RuntimeException(
                      "For an all-CTRE drive base, use Phoenix Tuner X Swerve Generator instead of YAGSL!");
                }
              case 0b00010000: // Blended Talon Drive / NEO Steer
                modules[i] = new Module(new ModuleIOBlended(i), i);
                break;
              case 0b01010000: // NEO motors + CANcoder
                modules[i] = new Module(new ModuleIOSparkCANcoder(i), i);
                break;
              case 0b01010100: // NEO motors + analog encoder
                modules[i] = new Module(new ModuleIOSpark(i), i);
                break;
              default:
                throw new RuntimeException("Invalid swerve module combination");
            }
          }
          break;

        default:
          throw new RuntimeException("Invalid Swerve Drive Type");
      }

      // Start odometry thread (for the real robot)
      PhoenixOdometryThread.getInstance().start();

    } else {

      // If SIM, just order up some SIM modules!
      for (int i = 0; i < 4; i++) {
        modules[i] = new Module(new ModuleIOSim(), i);
      }

      // Load the physics simulator
      simPhysics =
          new DriveSimPhysics(
              kinematics,
              RobotConstants.kRobotMOI, // kg m^2
              RobotConstants.kMaxWheelTorque); // Nm
    }

    // Usage reporting for swerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

    // Configure Autonomous Path Building for PathPlanner based on `AutoType`
    switch (Constants.getAutoType()) {
      case PATHPLANNER:
        try {
          // Configure AutoBuilder for PathPlanner
          AutoBuilder.configure(
              this::getPose,
              this::resetPose,
              this::getChassisSpeeds,
              (speeds, feedforwards) -> runVelocity(speeds),
              new PPHolonomicDriveController(
                  new PIDConstants(
                      DrivebaseConstants.kPStrafe,
                      DrivebaseConstants.kIStrafe,
                      DrivebaseConstants.kDStrafe),
                  new PIDConstants(
                      DrivebaseConstants.kPSPin,
                      DrivebaseConstants.kISPin,
                      DrivebaseConstants.kDSpin)),
              AutoConstants.kPathPlannerConfig,
              () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
              this);
        } catch (Exception e) {
          DriverStation.reportError(
              "Failed to load PathPlanner config and configure AutoBuilder", e.getStackTrace());
        }
        Pathfinding.setPathfinder(new LocalADStarAK());
        PathPlannerLogging.setLogActivePathCallback(
            (activePath) -> {
              Logger.recordOutput("Odometry/Trajectory", activePath.toArray(new Pose2d[0]));
            });
        PathPlannerLogging.setLogTargetPoseCallback(
            (targetPose) -> {
              Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
            });
        break;

      case CHOREO:
        // TODO: If your team is using Choreo, you'll know what to do here...
        break;

      case MANUAL:
        // Nothing to be done for MANUAL; may just use AutoPilot
        break;
      default:
    }

    // Configure SysId for drivebase characterization
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runCharacterization(voltage.in(Volts)), null, this));
  }

  /************************************************************************* */
  /** Periodic function that is called each robot cycle by the command scheduler */
  @Override
  public void rbsiPeriodic() {
    // NO odometryLock needed unless you touch poseEstimator directly here.
    // Keep this focused on control / setpoints / characterization.

    if (DriverStation.isDisabled()) {
      for (var module : modules) module.stop();
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // If you need module refresh for control, you can either:
    //  A) leave it in DriveOdometry (recommended) and here just use cached inputs, OR
    //  B) keep a lightweight "module.controlPeriodic()" that doesn't drain queues.
    //
    // For now: do nothing here re: odometry.
  }

  /**
   * Simulation Periodic Method
   *
   * <p>This function runs only for simulation, but does similar processing to the REAL periodic
   * function. Instead of reading back what the modules actually say, use physics to predict where
   * the module would have gone.
   */
  @Override
  public void simulationPeriodic() {
    final double dt = Constants.loopPeriodSecs;

    // Advance module wheel physics
    for (int i = 0; i < modules.length; i++) {
      modules[i].simulationPeriodic();
    }

    // Get module states from modules
    final SwerveModuleState[] moduleStates = new SwerveModuleState[modules.length];
    for (int i = 0; i < modules.length; i++) {
      moduleStates[i] = modules[i].getState();
    }

    // Update SIM physics (linear & angular motion of the robot)
    simPhysics.update(moduleStates, dt);

    // Feed the simulated IMU from authoritative physics
    final double yawRad = simPhysics.getYaw().getRadians();
    final double omegaRadPerSec = simPhysics.getOmegaRadPerSec();

    final double ax = simPhysics.getLinearAccel().getX();
    final double ay = simPhysics.getLinearAccel().getY();

    imu.simulationSetYawRad(yawRad);
    imu.simulationSetOmegaRadPerSec(omegaRadPerSec);
    imu.simulationSetLinearAccelMps2(ax, ay, 0.0);

    // Feed PoseEstimator with authoritative yaw and module positions
    final SwerveModulePosition[] modulePositions = new SwerveModulePosition[modules.length];
    for (int i = 0; i < modules.length; i++) {
      modulePositions[i] = modules[i].getPosition();
    }
    m_PoseEstimator.resetPosition(
        Rotation2d.fromRadians(yawRad), modulePositions, simPhysics.getPose());

    // If simulated vision available, inject vision measurement
    if (simulatedVisionAvailable) {
      final Pose2d visionPose = getSimulatedVisionPose();
      final double visionTimestamp = Timer.getFPGATimestamp();
      final var visionStdDevs = getSimulatedVisionStdDevs();
      m_PoseEstimator.addVisionMeasurement(visionPose, visionTimestamp, visionStdDevs);
    }

    poseBuffer.addSample(Timer.getFPGATimestamp(), simPhysics.getPose());

    // Logging
    Logger.recordOutput("Sim/Pose", simPhysics.getPose());
    Logger.recordOutput("Sim/YawRad", yawRad);
    Logger.recordOutput("Sim/OmegaRadPerSec", simPhysics.getOmegaRadPerSec());
    Logger.recordOutput("Sim/LinearAccelXY_mps2", new double[] {ax, ay});
  }

  /** Drive Base Action Functions ****************************************** */

  /**
   * Sets the swerve drive motors to brake/coast mode.
   *
   * @param brake True to set motors to brake mode, false for coast.
   */
  public void setMotorBrake(boolean brake) {
    {
      for (Module swerveModule : modules) {
        swerveModule.setBrakeMode(brake);
      }
    }
  }

  /** Stop the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, Constants.loopPeriodSecs);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, getMaxLinearSpeedMetersPerSec());

    // Log unoptimized setpoints and setpoint speeds
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }

    // Log optimized setpoints (runSetpoint mutates each state)
    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
  }

  /** Runs the drive in a straight line with the specified drive output. */
  public void runCharacterization(double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(output);
    }
  }

  /**
   * Reset the heading for the ProfiledPIDController
   *
   * <p>Call this when: (A) robot is disabled, (B) gyro is zeroed, (C) autonomous starts
   */
  public void resetHeadingController() {
    angleController.reset(getHeading().getRadians());
  }

  /** Getter function for the angle controller */
  public ProfiledPIDController getAngleController() {
    return angleController;
  }

  /** SysId Characterization Routines ************************************** */

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }

  /** Getter Functions ***************************************************** */
  public Module[] getModules() {
    return modules;
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Positions")
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /** Returns the measured chassis speeds of the robot. */
  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    if (Constants.getMode() == Mode.SIM) {
      return simPhysics.getPose();
    }
    return m_PoseEstimator.getEstimatedPosition();
  }

  /** Returns the current odometry rotation. */
  @AutoLogOutput(key = "Odometry/Yaw")
  public Rotation2d getHeading() {
    if (Constants.getMode() == Mode.SIM) {
      return simPhysics.getYaw();
    }
    return imu.getYaw();
  }

  /**
   * Returns the measured chassis speeds of the modules in FIELD coordinates.
   *
   * <p>+X = field forward +Y = field left CCW+ = counterclockwise
   */
  @AutoLogOutput(key = "SwerveChassisSpeeds/FieldMeasured")
  public ChassisSpeeds getFieldRelativeSpeeds() {
    // Robot-relative measured speeds from modules
    ChassisSpeeds robotRelative = getChassisSpeeds();
    // Convert to field-relative using authoritative yaw
    return ChassisSpeeds.fromRobotRelativeSpeeds(robotRelative, getHeading());
  }

  /**
   * Returns the FIELD-relative linear velocity of the robot's center.
   *
   * <p>+X = field forward +Y = field left
   */
  @AutoLogOutput(key = "Drive/FieldLinearVelocity")
  public Translation2d getFieldLinearVelocity() {
    ChassisSpeeds fieldSpeeds = getFieldRelativeSpeeds();
    return new Translation2d(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);
  }

  /** Returns interpolated odometry pose at a given timestamp. */
  public Optional<Pose2d> getPoseAtTime(double timestampSeconds) {
    return poseBuffer.getSample(timestampSeconds);
  }

  /**
   * Max abs yaw rate over [t0, t1] using buffered yaw-rate history
   *
   * @param t0 Interval start
   * @param t1 interval end
   * @return Maximum yaw rate
   */
  public OptionalDouble getMaxAbsYawRateRadPerSec(double t0, double t1) {
    // If end before start, return empty
    if (t1 < t0) return OptionalDouble.empty();

    // Get the subset of entries from the buffer
    var sub = yawRateBuffer.getInternalBuffer().subMap(t0, true, t1, true);
    if (sub.isEmpty()) return OptionalDouble.empty();

    double maxAbs = 0.0;
    boolean any = false;
    for (double v : sub.values()) {
      any = true;
      double a = Math.abs(v);
      if (a > maxAbs) maxAbs = a;
    }
    // Return a value if there's anything to report, else empty
    return any ? OptionalDouble.of(maxAbs) : OptionalDouble.empty();
  }

  /** Get the last EPOCH of a pose reset */
  public long getPoseResetEpoch() {
    return poseResetEpoch;
  }

  /** Get the last TIMESTAMP of a pose reset */
  public double getLastPoseResetTimestamp() {
    return lastPoseResetTimestamp;
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return DrivebaseConstants.kMaxLinearSpeed;
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return getMaxLinearSpeedMetersPerSec() / kDriveBaseRadiusMeters;
  }

  /** Returns the maximum linear acceleration in meters per sec per sec. */
  public double getMaxLinearAccelMetersPerSecPerSec() {
    return DrivebaseConstants.kMaxLinearAccel;
  }

  /** Returns the maximum angular acceleration in radians per sec per sec */
  public double getMaxAngularAccelRadPerSecPerSec() {
    return getMaxLinearAccelMetersPerSecPerSec() / kDriveBaseRadiusMeters;
  }

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(kFLXPosMeters, kFLYPosMeters),
      new Translation2d(kFRXPosMeters, kFRYPosMeters),
      new Translation2d(kBLXPosMeters, kBLYPosMeters),
      new Translation2d(kBRXPosMeters, kBRYPosMeters)
    };
  }

  /** Returns the position of each module in radians. */
  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  /** Returns the average velocity of the modules in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  /* Setter Functions ****************************************************** */

  /** Resets the current odometry pose. */
  public void resetPose(Pose2d pose) {
    m_PoseEstimator.resetPosition(getHeading(), getModulePositions(), pose);
    markPoseReset(Timer.getFPGATimestamp());
  }

  /** Zeros the gyro based on alliance color */
  public void zeroHeadingForAlliance() {
    imu.zeroYaw(
        DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
            ? Rotation2d.kZero
            : Rotation2d.k180deg);
    resetHeadingController();
    markPoseReset(Timer.getFPGATimestamp());
  }

  /** Zeros the heading */
  public void zeroHeading() {
    imu.zeroYaw(Rotation2d.kZero);
    resetHeadingController();
    markPoseReset(Timer.getFPGATimestamp());
  }

  /** Adds a vision measurement safely into the PoseEstimator. */
  public void addVisionMeasurement(TimedPose rbsiPose) {
    odometryLock.lock();
    try {
      m_PoseEstimator.addVisionMeasurement(
          rbsiPose.pose(), rbsiPose.timestampSeconds(), rbsiPose.stdDevs());
    } finally {
      odometryLock.unlock();
    }
  }

  /**
   * Sets the EPOCH and TIMESTAMP for a pose reset
   *
   * @param fpgaNow The FPGA timestamp of the pose reset
   */
  private void markPoseReset(double fpgaNow) {
    lastPoseResetTimestamp = fpgaNow;
    poseResetEpoch++;
    Logger.recordOutput("Drive/PoseResetEpoch", poseResetEpoch);
    Logger.recordOutput("Drive/PoseResetTimestamp", lastPoseResetTimestamp);
  }

  /** UTILITY FUNCTION SECTION ********************************************* */

  /** CHOREO SECTION (Ignore if AutoType == PATHPLANNER) ******************* */

  /** Choreo: Reset odometry */
  public Command resetOdometry(Pose2d orElseGet) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'resetOdometry'");
  }

  /** Swerve request to apply during field-centric path following */
  @SuppressWarnings("unused")
  private final SwerveRequest.ApplyFieldSpeeds m_pathApplyFieldSpeeds =
      new SwerveRequest.ApplyFieldSpeeds();

  // Choreo Controller Values
  private final PIDController m_pathXController = new PIDController(10, 0, 0);
  private final PIDController m_pathYController = new PIDController(10, 0, 0);
  private final PIDController m_pathThetaController = new PIDController(7, 0, 0);

  /**
   * Follows the given field-centric path sample with PID for Choreo
   *
   * @param pose Current pose of the robot
   * @param sample Sample along the path to follow
   */
  public void choreoController(Pose2d pose, SwerveSample sample) {
    m_pathThetaController.enableContinuousInput(-Math.PI, Math.PI);

    var targetSpeeds = sample.getChassisSpeeds();
    targetSpeeds.vxMetersPerSecond += m_pathXController.calculate(pose.getX(), sample.x);
    targetSpeeds.vyMetersPerSecond += m_pathYController.calculate(pose.getY(), sample.y);
    targetSpeeds.omegaRadiansPerSecond +=
        m_pathThetaController.calculate(pose.getRotation().getRadians(), sample.heading);

    // setControl(
    //     m_pathApplyFieldSpeeds
    //         .withSpeeds(targetSpeeds)
    //         .withWheelForceFeedforwardsX(sample.moduleForcesX())
    //         .withWheelForceFeedforwardsY(sample.moduleForcesY()));
  }

  public void followTrajectory(SwerveSample sample) {
    // Get the current pose of the robot
    Pose2d pose = getPose();

    // Generate the next speeds for the robot
    ChassisSpeeds speeds =
        new ChassisSpeeds(
            sample.vx + m_pathXController.calculate(pose.getX(), sample.x),
            sample.vy + m_pathXController.calculate(pose.getX(), sample.y),
            sample.omega
                + m_pathXController.calculate(pose.getRotation().getRadians(), sample.heading));

    // Apply the generated speeds
    runVelocity(speeds);
  }

  /** SIMULATION VISION FUNCTIONS ****************************************** */

  // Vision measurement enabled in simulation
  private boolean simulatedVisionAvailable = true;

  // Maximum simulated noise in meters/radians
  private static final double SIM_VISION_POS_NOISE_M = 0.02; // +/- 2cm
  private static final double SIM_VISION_YAW_NOISE_RAD = Math.toRadians(2); // +/- 2 degrees

  /**
   * Returns a simulated Pose2d for vision in field coordinates. Adds a small random jitter to
   * simulate measurement error.
   */
  private Pose2d getSimulatedVisionPose() {
    Pose2d truePose = simPhysics.getPose(); // authoritative pose

    // Add small random noise
    double dx = (Math.random() * 2 - 1) * SIM_VISION_POS_NOISE_M;
    double dy = (Math.random() * 2 - 1) * SIM_VISION_POS_NOISE_M;
    double dTheta = (Math.random() * 2 - 1) * SIM_VISION_YAW_NOISE_RAD;

    return new Pose2d(
        truePose.getX() + dx,
        truePose.getY() + dy,
        truePose.getRotation().plus(new Rotation2d(dTheta)));
  }

  /**
   * Returns the standard deviations for the simulated vision measurement. These values are used by
   * the PoseEstimator to weight vision updates.
   */
  private edu.wpi.first.math.Matrix<N3, N1> getSimulatedVisionStdDevs() {
    edu.wpi.first.math.Matrix<N3, N1> stdDevs =
        new edu.wpi.first.math.Matrix<>(N3.instance, N1.instance);
    stdDevs.set(0, 0, 0.02); // X standard deviation (meters)
    stdDevs.set(1, 0, 0.02); // Y standard deviation (meters)
    stdDevs.set(2, 0, Math.toRadians(2)); // rotation standard deviation (radians)
    return stdDevs;
  }

  // --- Helpers used by DriveOdometry (package-private) ---

  Pose2d poseEstimatorGetPose() {
    return m_PoseEstimator.getEstimatedPosition();
  }

  void poseEstimatorUpdateWithTime(double t, Rotation2d yaw, SwerveModulePosition[] positions) {
    m_PoseEstimator.updateWithTime(t, yaw, positions);
  }

  void poseBufferAddSample(double t, Pose2d pose) {
    poseBuffer.addSample(t, pose);
  }

  // Yaw buffer helpers (assuming you already have yawBuffer + yawRateBuffer)
  double yawBufferSampleOr(double t, double fallbackYawRad) {
    return yawBuffer.getSample(t).orElse(fallbackYawRad);
  }

  void yawBuffersAddSample(double t, double yawRad, double yawRateRadPerSec) {
    yawBuffer.addSample(t, yawRad);
    yawRateBuffer.addSample(t, yawRateRadPerSec);
  }

  void yawBuffersFillFromQueue(double[] yawTs, double[] yawPosRad) {
    for (int k = 0; k < yawTs.length; k++) {
      yawBuffer.addSample(yawTs[k], yawPosRad[k]);
      if (k > 0) {
        double dt = yawTs[k] - yawTs[k - 1];
        if (dt > 1e-6) {
          yawRateBuffer.addSample(yawTs[k], (yawPosRad[k] - yawPosRad[k - 1]) / dt);
        }
      }
    }
  }

  void yawBuffersAddSampleIndexAligned(double t, double[] yawTs, double[] yawPos, int i) {
    yawBuffer.addSample(t, yawPos[i]);
    if (i > 0) {
      double dt = yawTs[i] - yawTs[i - 1];
      if (dt > 1e-6) {
        yawRateBuffer.addSample(t, (yawPos[i] - yawPos[i - 1]) / dt);
      }
    }
  }

  void setGyroDisconnectedAlert(boolean disconnected) {
    gyroDisconnectedAlert.set(disconnected);
  }

  // Drive.java
  public Pose2d getSimPose() {
    return simPhysics.getPose();
  }

  public double getSimYawRad() {
    return simPhysics.getYaw().getRadians();
  }

  public double getSimYawRateRadPerSec() {
    return simPhysics.getOmegaRadPerSec();
  }
}
