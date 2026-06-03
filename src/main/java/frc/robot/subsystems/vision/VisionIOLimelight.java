// Copyright (c) 2024-2026 Az-FIRST
// http://github.com/AZ-First
// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the AdvantageKit-License.md file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.wpilibj.RobotController;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;

/** IO implementation for real Limelight hardware. */
public class VisionIOLimelight implements VisionIO {
  private static final int BOTPOSE_MIN_LENGTH = 11;
  private static final int BOTPOSE_LATENCY_INDEX = 6;
  private static final int BOTPOSE_TAG_COUNT_INDEX = 7;
  private static final int BOTPOSE_AVG_DISTANCE_INDEX = 9;
  private static final int BOTPOSE_FIRST_TAG_ID_INDEX = 11;
  private static final int BOTPOSE_FIRST_TAG_AMBIGUITY_INDEX = 17;
  private static final int BOTPOSE_TAG_STRIDE = 7;
  private static final long ORIENTATION_FLUSH_PERIOD_US = 20_000;
  private static long lastOrientationFlushUs = 0;

  private final Supplier<Rotation2d> rotationSupplier;
  private final DoubleArrayPublisher orientationPublisher;

  private final DoubleSubscriber heartbeatSubscriber;
  private final DoubleSubscriber txSubscriber;
  private final DoubleSubscriber tySubscriber;
  private final DoubleArraySubscriber megatag1Subscriber;
  private final DoubleArraySubscriber megatag2Subscriber;

  /**
   * Creates a new VisionIOLimelight.
   *
   * @param name The configured name of the Limelight.
   * @param rotationSupplier Supplier for the current estimated rotation, used for MegaTag 2.
   */
  public VisionIOLimelight(String name, Supplier<Rotation2d> rotationSupplier) {
    var table = NetworkTableInstance.getDefault().getTable(name);
    this.rotationSupplier = rotationSupplier;
    orientationPublisher = table.getDoubleArrayTopic("robot_orientation_set").publish();
    heartbeatSubscriber = table.getDoubleTopic("hb").subscribe(0.0);
    txSubscriber = table.getDoubleTopic("tx").subscribe(0.0);
    tySubscriber = table.getDoubleTopic("ty").subscribe(0.0);
    megatag1Subscriber = table.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[] {});
    megatag2Subscriber =
        table.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[] {});
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    // Update connection status based on whether an update has been seen in the last 250ms
    inputs.connected =
        ((RobotController.getFPGATime() - heartbeatSubscriber.getLastChange()) / 1000) < 250;

    // Update target observation
    inputs.latestTargetObservation =
        new TargetObservation(
            Rotation2d.fromDegrees(txSubscriber.get()), Rotation2d.fromDegrees(tySubscriber.get()));

    // Update orientation for MegaTag 2
    orientationPublisher.accept(
        new double[] {rotationSupplier.get().getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0});
    flushOrientationIfDue();

    // Read new pose observations from NetworkTables
    Set<Integer> unionTagIds = new HashSet<>();
    List<PoseObservation> poseObservations = new ArrayList<>();

    for (var rawSample : megatag1Subscriber.readQueue()) {
      if (!isValidBotPoseSample(rawSample.value)) continue;

      int tagCount = getTagCount(rawSample.value);
      int[] used = extractUsedTagIds(rawSample.value);
      for (int id : used) unionTagIds.add(id);

      poseObservations.add(
          new PoseObservation(
              // Timestamp, based on server timestamp of publish and latency
              timestampSeconds(rawSample),

              // 3D pose estimate
              parsePose(rawSample.value),

              // Ambiguity, using only the first tag because ambiguity isn't applicable for multitag
              rawSample.value.length > BOTPOSE_FIRST_TAG_AMBIGUITY_INDEX
                  ? rawSample.value[BOTPOSE_FIRST_TAG_AMBIGUITY_INDEX]
                  : 0.0,

              // Tag count
              tagCount,

              // Average tag distance
              rawSample.value[BOTPOSE_AVG_DISTANCE_INDEX],

              // Observation type
              PoseObservationType.MEGATAG_1,

              // Used tag IDs
              used));
    }

    for (var rawSample : megatag2Subscriber.readQueue()) {
      if (!isValidBotPoseSample(rawSample.value)) continue;

      int tagCount = getTagCount(rawSample.value);
      int[] used = extractUsedTagIds(rawSample.value);
      for (int id : used) unionTagIds.add(id);

      poseObservations.add(
          new PoseObservation(
              // Timestamp, based on server timestamp of publish and latency
              timestampSeconds(rawSample),

              // 3D pose estimate
              parsePose(rawSample.value),

              // Ambiguity, zeroed because the pose is already disambiguated
              0.0,

              // Tag count
              tagCount,

              // Average tag distance
              rawSample.value[BOTPOSE_AVG_DISTANCE_INDEX],

              // Observation type
              PoseObservationType.MEGATAG_2,
              used));
    }

    // Save pose observations to inputs object
    inputs.poseObservations = poseObservations.toArray(new PoseObservation[0]);

    inputs.tagIds = new int[unionTagIds.size()];
    int i = 0;
    for (int id : unionTagIds) {
      inputs.tagIds[i++] = id;
    }

    // Sort list by TagID for clarity
    Arrays.sort(inputs.tagIds);
  }

  /** Parses the 3D pose from a Limelight botpose array. */
  static Pose3d parsePose(double[] rawLLArray) {
    if (rawLLArray.length < BOTPOSE_MIN_LENGTH) {
      throw new IllegalArgumentException(
          "Limelight botpose array must have at least " + BOTPOSE_MIN_LENGTH + " values.");
    }
    return new Pose3d(
        rawLLArray[0],
        rawLLArray[1],
        rawLLArray[2],
        new Rotation3d(
            Units.degreesToRadians(rawLLArray[3]),
            Units.degreesToRadians(rawLLArray[4]),
            Units.degreesToRadians(rawLLArray[5])));
  }

  static boolean isValidBotPoseSample(double[] rawLLArray) {
    return rawLLArray.length >= BOTPOSE_MIN_LENGTH
        && getTagCount(rawLLArray) >= 0
        && Double.isFinite(rawLLArray[BOTPOSE_LATENCY_INDEX])
        && Double.isFinite(rawLLArray[BOTPOSE_AVG_DISTANCE_INDEX]);
  }

  static int[] extractUsedTagIds(double[] rawLLArray) {
    int tagCount = getTagCount(rawLLArray);
    int[] used = new int[Math.max(0, tagCount)];
    int count = 0;

    for (int i = BOTPOSE_FIRST_TAG_ID_INDEX;
        i < rawLLArray.length && count < used.length;
        i += BOTPOSE_TAG_STRIDE) {
      used[count++] = (int) rawLLArray[i];
    }

    return count == used.length ? used : Arrays.copyOf(used, count);
  }

  static double timestampSeconds(TimestampedDoubleArray rawSample) {
    return timestampSeconds(rawSample.timestamp, rawSample.value);
  }

  static double timestampSeconds(long ntTimestampMicros, double[] rawLLArray) {
    return ntTimestampMicros * 1.0e-6 - rawLLArray[BOTPOSE_LATENCY_INDEX] * 1.0e-3;
  }

  private static int getTagCount(double[] rawLLArray) {
    return (int) rawLLArray[BOTPOSE_TAG_COUNT_INDEX];
  }

  private static void flushOrientationIfDue() {
    long nowUs = RobotController.getFPGATime();
    if (nowUs - lastOrientationFlushUs < ORIENTATION_FLUSH_PERIOD_US) return;

    NetworkTableInstance.getDefault().flush();
    lastOrientationFlushUs = nowUs;
  }
}
