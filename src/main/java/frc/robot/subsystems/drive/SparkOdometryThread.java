// Copyright (c) 2024-2026 Az-FIRST
// http://github.com/AZ-First
// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the AdvantageKit-License.md file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import org.wpilib.system.Notifier;
import org.wpilib.system.RobotController;

/**
 * Provides an interface for asynchronously reading high-frequency measurements to a set of queues.
 *
 * <p>This version includes an overload for Spark signals, which checks for errors to ensure that
 * all measurements in the sample are valid.
 */
public class SparkOdometryThread {
  private static final int QUEUE_CAPACITY = 128;

  private final List<SparkBase> sparks = new ArrayList<>();
  private final List<DoubleSupplier> sparkSignals = new ArrayList<>();
  private final List<DoubleSupplier> genericSignals = new ArrayList<>();
  private final List<Queue<Double>> sparkQueues = new ArrayList<>();
  private final List<Queue<Double>> genericQueues = new ArrayList<>();
  private final List<Queue<Double>> timestampQueues = new ArrayList<>();

  private static SparkOdometryThread instance = null;
  private final Notifier notifier = new Notifier(this::run);
  private long droppedSamples = 0;
  private long loopCount = 0;

  public static SparkOdometryThread getInstance() {
    if (instance == null) {
      instance = new SparkOdometryThread();
    }
    return instance;
  }

  private SparkOdometryThread() {
    notifier.setName("OdometryThread");
  }

  public void start() {
    if (!timestampQueues.isEmpty()) {
      notifier.startPeriodic(1.0 / SwerveConstants.kOdometryFrequency);
    }
  }

  /** Registers a Spark signal to be read from the thread. */
  public Queue<Double> registerSignal(SparkBase spark, DoubleSupplier signal) {
    Queue<Double> queue = createQueue();
    Drive.odometryLock.lock();
    try {
      sparks.add(spark);
      sparkSignals.add(signal);
      sparkQueues.add(queue);
    } finally {
      Drive.odometryLock.unlock();
    }
    return queue;
  }

  /** Registers a generic signal to be read from the thread. */
  public Queue<Double> registerSignal(DoubleSupplier signal) {
    Queue<Double> queue = createQueue();
    Drive.odometryLock.lock();
    try {
      genericSignals.add(signal);
      genericQueues.add(queue);
    } finally {
      Drive.odometryLock.unlock();
    }
    return queue;
  }

  /** Returns a new queue that returns timestamp values for each sample. */
  public Queue<Double> makeTimestampQueue() {
    Queue<Double> queue = createQueue();
    Drive.odometryLock.lock();
    try {
      timestampQueues.add(queue);
    } finally {
      Drive.odometryLock.unlock();
    }
    return queue;
  }

  private void run() {
    // Save new data to queues
    Drive.odometryLock.lock();
    try {
      // Get sample timestamp
      double timestamp = RobotController.getTime() / 1e6;

      // Read Spark values, mark invalid in case of error
      double[] sparkValues = new double[sparkSignals.size()];
      boolean isValid = true;
      for (int i = 0; i < sparkSignals.size(); i++) {
        sparkValues[i] = sparkSignals.get(i).getAsDouble();
        if (sparks.get(i).getLastError() != REVLibError.kOk) {
          isValid = false;
        }
      }

      // If valid, add values to queues
      if (isValid) {
        for (int i = 0; i < sparkSignals.size(); i++) {
          offerSample(sparkQueues.get(i), sparkValues[i]);
        }
        for (int i = 0; i < genericSignals.size(); i++) {
          offerSample(genericQueues.get(i), genericSignals.get(i).getAsDouble());
        }
        for (Queue<Double> timestampQueue : timestampQueues) {
          offerSample(timestampQueue, timestamp);
        }
      }
    } finally {
      Drive.odometryLock.unlock();
    }

    if ((loopCount++ % (int) SwerveConstants.kOdometryFrequency) == 0) {
      Logger.recordOutput("Drive/SparkOdomThread/DroppedSamples", droppedSamples);
    }
  }

  private static Queue<Double> createQueue() {
    return new ArrayBlockingQueue<>(QUEUE_CAPACITY);
  }

  private void offerSample(Queue<Double> queue, double sample) {
    if (!queue.offer(sample)) {
      droppedSamples++;
    }
  }
}
