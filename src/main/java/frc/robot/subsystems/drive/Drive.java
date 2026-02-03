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
import edu.wpi.first.math.Matrix;
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
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends RBSISubsystem {

  // --- buffers for time-aligned pose + yaw + yawRate history ---
  private final ConcurrentTimeInterpolatableBuffer<Pose2d> poseBuffer =
      frc.robot.util.ConcurrentTimeInterpolatableBuffer.createBuffer(1.5);

  private final ConcurrentTimeInterpolatableBuffer<Double> yawBuffer =
      frc.robot.util.ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(1.5);

  private final ConcurrentTimeInterpolatableBuffer<Double> yawRateBuffer =
      frc.robot.util.ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(1.5);

  static final Lock odometryLock = new ReentrantLock();
  private final Imu imu;
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final SysIdRoutine sysId;
  private final Alert gyroDisconnectedAlert =
      new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private final SwerveModulePosition[] odomPositions = new SwerveModulePosition[4];
  private SwerveDrivePoseEstimator m_PoseEstimator =
      new SwerveDrivePoseEstimator(kinematics, Rotation2d.kZero, lastModulePositions, Pose2d.kZero);

  private ProfiledPIDController angleController;

  private DriveSimPhysics simPhysics;

  // Constructor
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
  public void rbsiPeriodic() {
    odometryLock.lock();
    try {
      // Get the IMU inputs
      final var imuInputs = imu.getInputs();

      // Stop modules & log empty setpoint states if disabled
      if (DriverStation.isDisabled()) {
        for (var module : modules) module.stop();
        Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
        Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
      }

      // Module periodic updates, which drains queues this cycle
      for (var module : modules) module.periodic();

      // Feed historical samples into odometry if REAL robot
      if (Constants.getMode() != Mode.SIM) {

        // ---- Gather per-module histories ----
        final double[][] perModuleTs = new double[4][];
        final SwerveModulePosition[][] perModulePos = new SwerveModulePosition[4][];
        int unionCap = 0;

        for (int m = 0; m < 4; m++) {
          perModuleTs[m] = modules[m].getOdometryTimestamps();
          perModulePos[m] = modules[m].getOdometryPositions();
          unionCap += perModuleTs[m].length;
        }

        if (unionCap == 0) {
          gyroDisconnectedAlert.set(!imuInputs.connected);
          return;
        }

        // ---- Fill yaw buffer from IMU odometry samples (preferred) ----
        // These timestamps are authoritative for yaw interpolation and yaw-rate gating.
        if (imuInputs.connected
            && imuInputs.odometryYawTimestamps != null
            && imuInputs.odometryYawPositionsRad != null
            && imuInputs.odometryYawTimestamps.length == imuInputs.odometryYawPositionsRad.length
            && imuInputs.odometryYawTimestamps.length > 0) {

          final double[] yts = imuInputs.odometryYawTimestamps;
          final double[] yaws = imuInputs.odometryYawPositionsRad;

          for (int i = 0; i < yts.length; i++) {
            yawBuffer.addSample(yts[i], yaws[i]);
            if (i > 0) {
              double dt = yts[i] - yts[i - 1];
              if (dt > 1e-6) {
                yawRateBuffer.addSample(yts[i], (yaws[i] - yaws[i - 1]) / dt);
              }
            }
          }
        } else {
          // fallback: buffer "now" yaw only
          double now = Timer.getFPGATimestamp();
          yawBuffer.addSample(now, imuInputs.yawPositionRad);
          yawRateBuffer.addSample(now, imuInputs.yawRateRadPerSec);
        }

        // ---- Build union timeline ----
        final double[] unionTs = new double[Math.max(unionCap, 1)];
        final double EPS = 1e-4; // 0.1 ms
        final int unionN = buildUnionTimeline(perModuleTs, unionTs, EPS);

        // ---- Replay at union times ----
        for (int i = 0; i < unionN; i++) {
          final double t = unionTs[i];

          // Interpolate each module to union time
          for (int m = 0; m < 4; m++) {
            SwerveModulePosition p = interpolateModulePosition(perModuleTs[m], perModulePos[m], t);
            odomPositions[m] = p;
          }

          // Interpolate yaw at union time
          final double yawRad = yawBuffer.getSample(t).orElse(imuInputs.yawPositionRad);

          m_PoseEstimator.updateWithTime(t, Rotation2d.fromRadians(yawRad), odomPositions);

          // keep buffer in the same timebase as estimator
          poseBuffer.addSample(t, m_PoseEstimator.getEstimatedPosition());
        }

        Logger.recordOutput("Drive/Pose", m_PoseEstimator.getEstimatedPosition());
      }

      gyroDisconnectedAlert.set(!imuInputs.connected && Constants.getMode() != Mode.SIM);

    } finally {
      odometryLock.unlock();
    }
  }

  /** Simulation Periodic Method */
  @Override
  public void simulationPeriodic() {
    final double dt = Constants.loopPeriodSecs;

    // 1) Advance module wheel physics
    for (int i = 0; i < modules.length; i++) {
      modules[i].simulationPeriodic();
    }

    // 2) Get module states from modules (authoritative) - NO STREAMS
    final SwerveModuleState[] moduleStates = new SwerveModuleState[modules.length];
    for (int i = 0; i < modules.length; i++) {
      moduleStates[i] = modules[i].getState();
    }

    // 3) Update SIM physics (linear + angular)
    simPhysics.update(moduleStates, dt);

    // 4) Feed IMU from authoritative physics (primitive-only boundary)
    final double yawRad =
        simPhysics.getYaw().getRadians(); // or simPhysics.getYawRad() if you have it
    final double omegaRadPerSec = simPhysics.getOmegaRadPerSec();

    final double ax = simPhysics.getLinearAccel().getX();
    final double ay = simPhysics.getLinearAccel().getY();

    imu.simulationSetYawRad(yawRad);
    imu.simulationSetOmegaRadPerSec(omegaRadPerSec);
    imu.simulationSetLinearAccelMps2(ax, ay, 0.0);

    // 5) Feed PoseEstimator with authoritative yaw and module positions
    // (PoseEstimator still wants objects -> boundary conversion stays here)
    final SwerveModulePosition[] modulePositions = new SwerveModulePosition[modules.length];
    for (int i = 0; i < modules.length; i++) {
      modulePositions[i] = modules[i].getPosition();
    }

    m_PoseEstimator.resetPosition(
        Rotation2d.fromRadians(yawRad), modulePositions, simPhysics.getPose());

    // 6) Optional: inject vision measurement in SIM
    if (simulatedVisionAvailable) {
      final Pose2d visionPose = getSimulatedVisionPose();
      final double visionTimestamp = Timer.getFPGATimestamp();
      final var visionStdDevs = getSimulatedVisionStdDevs();
      m_PoseEstimator.addVisionMeasurement(visionPose, visionTimestamp, visionStdDevs);
    }

    poseBuffer.addSample(Timer.getFPGATimestamp(), simPhysics.getPose());

    // 7) Logging
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

  /** Returns interpolated odometry pose at a given timestamp. */
  public Optional<Pose2d> getPoseAtTime(double timestampSeconds) {
    return poseBuffer.getSample(timestampSeconds);
  }

  /** Adds a vision measurement safely into the PoseEstimator. */
  public void addVisionMeasurement(Pose2d pose, double timestampSeconds, Matrix<N3, N1> stdDevs) {
    odometryLock.lock();
    try {
      m_PoseEstimator.addVisionMeasurement(pose, timestampSeconds, stdDevs);
    } finally {
      odometryLock.unlock();
    }
  }

  /** Max abs yaw rate over [t0, t1] using buffered yaw-rate history (no streams). */
  public OptionalDouble getMaxAbsYawRateRadPerSec(double t0, double t1) {
    if (t1 < t0) return OptionalDouble.empty();
    var sub = yawRateBuffer.getInternalBuffer().subMap(t0, true, t1, true);
    if (sub.isEmpty()) return OptionalDouble.empty();

    double maxAbs = 0.0;
    boolean any = false;
    for (double v : sub.values()) {
      any = true;
      double a = Math.abs(v);
      if (a > maxAbs) maxAbs = a;
    }
    return any ? OptionalDouble.of(maxAbs) : OptionalDouble.empty();
  }

  /** Returns the current odometry rotation. */
  @AutoLogOutput(key = "Odometry/Yaw")
  public Rotation2d getHeading() {
    if (Constants.getMode() == Mode.SIM) {
      return simPhysics.getYaw();
    }
    return imu.getYaw();
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

  /** Returns the maximum linear acceleration in meters per sec per sec. */
  public double getMaxLinearAccelMetersPerSecPerSec() {
    return DrivebaseConstants.kMaxLinearAccel;
  }

  /** Returns the maximum angular acceleration in radians per sec per sec */
  public double getMaxAngularAccelRadPerSecPerSec() {
    return getMaxLinearAccelMetersPerSecPerSec() / kDriveBaseRadiusMeters;
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

  // ---- Pose reset gate (vision + anything latency-sensitive) ----
  private volatile long poseResetEpoch = 0; // monotonic counter
  private volatile double lastPoseResetTimestamp = Double.NEGATIVE_INFINITY;

  public long getPoseResetEpoch() {
    return poseResetEpoch;
  }

  public double getLastPoseResetTimestamp() {
    return lastPoseResetTimestamp;
  }

  private void markPoseReset(double fpgaNow) {
    lastPoseResetTimestamp = fpgaNow;
    poseResetEpoch++;
    Logger.recordOutput("Drive/PoseResetEpoch", poseResetEpoch);
    Logger.recordOutput("Drive/PoseResetTimestamp", lastPoseResetTimestamp);
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

  private static SwerveModulePosition interpolateModulePosition(
      double[] ts, SwerveModulePosition[] samples, double t) {

    final int n = ts.length;
    if (n == 0) return new SwerveModulePosition();

    if (t <= ts[0]) return samples[0];
    if (t >= ts[n - 1]) return samples[n - 1];

    int lo = 0, hi = n - 1;
    while (lo < hi) {
      int mid = (lo + hi) >>> 1;
      if (ts[mid] < t) lo = mid + 1;
      else hi = mid;
    }
    int i1 = lo;
    int i0 = i1 - 1;

    double t0 = ts[i0];
    double t1 = ts[i1];
    if (t1 <= t0) return samples[i1];

    double alpha = (t - t0) / (t1 - t0);

    double dist =
        samples[i0].distanceMeters
            + (samples[i1].distanceMeters - samples[i0].distanceMeters) * alpha;

    Rotation2d angle = samples[i0].angle.interpolate(samples[i1].angle, alpha);

    return new SwerveModulePosition(dist, angle);
  }

  private static int buildUnionTimeline(double[][] perModuleTs, double[] outUnion, double epsSec) {
    int[] idx = new int[perModuleTs.length];
    int outN = 0;

    while (true) {
      double minT = Double.POSITIVE_INFINITY;
      boolean any = false;

      for (int m = 0; m < perModuleTs.length; m++) {
        double[] ts = perModuleTs[m];
        if (idx[m] >= ts.length) continue;
        any = true;
        minT = Math.min(minT, ts[idx[m]]);
      }

      if (!any) break;

      if (outN == 0 || Math.abs(minT - outUnion[outN - 1]) > epsSec) {
        outUnion[outN++] = minT;
      }

      for (int m = 0; m < perModuleTs.length; m++) {
        double[] ts = perModuleTs[m];
        while (idx[m] < ts.length && Math.abs(ts[idx[m]] - minT) <= epsSec) {
          idx[m]++;
        }
      }
    }

    return outN;
  }
}
