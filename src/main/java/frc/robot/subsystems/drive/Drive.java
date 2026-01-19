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
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.subsystems.imu.ImuIO;
import frc.robot.util.LocalADStarAK;
import frc.robot.util.RBSIEnum.Mode;
import frc.robot.util.RBSIParsing;
import java.util.Arrays;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {

  static final Lock odometryLock = new ReentrantLock();
  private final ImuIO imuIO;
  private final ImuIO.ImuIOInputs imuInputs = new ImuIO.ImuIOInputs();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final SysIdRoutine sysId;
  private final Alert gyroDisconnectedAlert =
      new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
  private Rotation2d rawGyroRotation = imuInputs.yawPosition;
  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private SwerveDrivePoseEstimator m_PoseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, Pose2d.kZero);

  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(
          DrivebaseConstants.kPTheta,
          DrivebaseConstants.kITheta,
          DrivebaseConstants.kDTheta,
          new TrapezoidProfile.Constraints(
              DrivebaseConstants.kMaxAngularSpeed, DrivebaseConstants.kMaxAngularAccel));

  private DriveSimPhysics simPhysics;

  // Constructor
  public Drive(ImuIO imuIO) {
    this.imuIO = imuIO;

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
                if (kImuType == "navx" || kImuType == "navx_spi") {
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
              new PPHolonomicDriveController(AutoConstants.kPPdrivePID, AutoConstants.kPPsteerPID),
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
        // TODO: Probably need to add something here for Choreo autonomous path building
        break;
      default:
    }

    // Configure SysId
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

  /** Periodic function that is called each robot cycle by the command scheduler */
  @Override
  public void periodic() {
    odometryLock.lock();

    // Stop modules & log empty setpoint states if disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
        Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
        Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
      }
    }

    // Update the IMU inputs -- logging happens automatically
    imuIO.updateInputs(imuInputs);

    // Feed historical samples into odometry if REAL robot
    if (Constants.getMode() != Mode.SIM) {
      double[] sampleTimestamps = modules[0].getOdometryTimestamps();
      int sampleCount = sampleTimestamps.length;

      for (int i = 0; i < sampleCount; i++) {
        // Read wheel positions and deltas from each module
        SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
        SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];

        for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
          modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
          moduleDeltas[moduleIndex] =
              new SwerveModulePosition(
                  modulePositions[moduleIndex].distanceMeters
                      - lastModulePositions[moduleIndex].distanceMeters,
                  modulePositions[moduleIndex].angle);
          lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
        }

        // Update gyro angle for odometry
        Rotation2d yaw =
            imuInputs.connected && imuInputs.odometryYawPositions.length > i
                ? imuInputs.odometryYawPositions[i]
                : imuInputs.yawPosition;

        // Apply to pose estimator
        m_PoseEstimator.updateWithTime(sampleTimestamps[i], yaw, modulePositions);
      }
      Logger.recordOutput("Drive/Pose", m_PoseEstimator.getEstimatedPosition());
    }

    // Module periodic updates
    for (var module : modules) {
      module.periodic();
    }

    odometryLock.unlock();

    // Update gyro/IMU alert
    gyroDisconnectedAlert.set(!imuInputs.connected && Constants.getMode() != Mode.SIM);
  }

  /** Simulation Periodic Method */
  @Override
  public void simulationPeriodic() {
    final double dt = Constants.loopPeriodSecs;

    // 1) Advance module wheel physics
    for (Module module : modules) {
      module.simulationPeriodic();
    }

    // 2) Get module states from modules (authoritative)
    SwerveModuleState[] moduleStates =
        Arrays.stream(modules).map(Module::getState).toArray(SwerveModuleState[]::new);

    // 3) Update SIM physics (linear + angular)
    simPhysics.update(moduleStates, dt);

    // 4) Feed IMU from authoritative physics
    imuIO.simulationSetYaw(simPhysics.getYaw());
    imuIO.simulationSetOmega(simPhysics.getOmegaRadPerSec());
    imuIO.setLinearAccel(
        new Translation3d(
            simPhysics.getLinearAccel().getX(), simPhysics.getLinearAccel().getY(), 0.0));

    // 5) Feed PoseEstimator with authoritative yaw and module positions
    SwerveModulePosition[] modulePositions =
        Arrays.stream(modules).map(Module::getPosition).toArray(SwerveModulePosition[]::new);

    m_PoseEstimator.resetPosition(
        simPhysics.getYaw(), // gyro reading (authoritative)
        modulePositions, // wheel positions
        simPhysics.getPose() // pose is authoritative
        );

    // 6) Optional: inject vision measurement in SIM
    if (simulatedVisionAvailable) {
      Pose2d visionPose = getSimulatedVisionPose();
      double visionTimestamp = Timer.getFPGATimestamp();
      var visionStdDevs = getSimulatedVisionStdDevs();
      m_PoseEstimator.addVisionMeasurement(visionPose, visionTimestamp, visionStdDevs);
    }

    // 7) Logging
    Logger.recordOutput("Sim/Pose", simPhysics.getPose());
    Logger.recordOutput("Sim/Yaw", simPhysics.getYaw());
    Logger.recordOutput("Sim/LinearAccel", simPhysics.getLinearAccel());
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

  /** Stops the drive. */
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
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, DrivebaseConstants.kMaxLinearSpeed);

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
   * Reset the heading ProfiledPIDController
   *
   * <p>TODO: CALL THIS FUNCTION!!!
   *
   * <p>Call this when: (A) robot is disabled, (B) gyro is zeroed, (C) autonomous starts
   */
  public void resetHeadingController() {
    thetaController.reset(getHeading().getRadians());
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
    return imuInputs.yawPosition;
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

  /**
   * Returns the measured chassis speeds in FIELD coordinates.
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

  /** Returns the average velocity of the modules in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return DrivebaseConstants.kMaxLinearSpeed;
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return getMaxLinearSpeedMetersPerSec() / kDriveBaseRadiusMeters;
  }

  /* Setter Functions ****************************************************** */

  /** Resets the current odometry pose. */
  public void resetPose(Pose2d pose) {
    m_PoseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  /** Zeros the gyro based on alliance color */
  public void zeroHeadingForAlliance() {
    imuIO.zeroYaw(
        DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
            ? Rotation2d.kZero
            : Rotation2d.k180deg);
  }

  /** Zeros the heading */
  public void zeroHeading() {
    imuIO.zeroYaw(Rotation2d.kZero);
  }

  /** Adds a new timestamped vision measurement. */
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    m_PoseEstimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

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

  // ---------------- SIM VISION ----------------

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
}
