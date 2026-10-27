// Copyright (c) 2024-2026 Az-FIRST
// http://github.com/AZ-First
// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the AdvantageKit-License.md file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Queue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.system.RobotController;
import org.wpilib.units.measure.Angle;

/**
 * Provides an interface for asynchronously reading high-frequency measurements to a set of queues.
 *
 * <p>Signals on one CAN FD bus use the "waitForAll" blocking method. When signals span multiple
 * buses, they are refreshed in separate per-bus groups because Phoenix bulk calls reject mixed
 * networks. The resulting shared timestamp is an approximation, not cross-bus synchronization.
 */
public class PhoenixOdometryThread extends Thread {
  private static final int QUEUE_CAPACITY = 128;

  private final Lock signalsLock =
      new ReentrantLock(); // Prevents conflicts when registering signals
  private BaseStatusSignal[] phoenixSignals = new BaseStatusSignal[0];
  private final List<String> phoenixSignalBuses = new ArrayList<>();
  private Map<String, BaseStatusSignal[]> phoenixSignalsByBus = Map.of();
  private boolean canWaitForAll = false;
  private final List<DoubleSupplier> genericSignals = new ArrayList<>();
  private final List<LatestSampleQueue<Double>> phoenixQueues = new ArrayList<>();
  private final List<LatestSampleQueue<Double>> genericQueues = new ArrayList<>();
  private final List<LatestSampleQueue<Double>> timestampQueues = new ArrayList<>();

  private static PhoenixOdometryThread instance = null;

  private long droppedSamples = 0;
  private long loopCount = 0;

  public static PhoenixOdometryThread getInstance() {
    if (instance == null) {
      instance = new PhoenixOdometryThread();
    }
    return instance;
  }

  private PhoenixOdometryThread() {
    setName("PhoenixOdometryThread");
    setDaemon(true);
  }

  @Override
  public void start() {
    if (!timestampQueues.isEmpty()) {
      super.start();
    }
  }

  /** Registers a Phoenix signal to be read from the thread. */
  public Queue<Double> registerSignal(String busName, StatusSignal<Angle> signal) {
    LatestSampleQueue<Double> queue = createQueue();
    signalsLock.lock();
    Drive.odometryLock.lock();
    try {
      BaseStatusSignal[] newSignals = new BaseStatusSignal[phoenixSignals.length + 1];
      System.arraycopy(phoenixSignals, 0, newSignals, 0, phoenixSignals.length);
      newSignals[phoenixSignals.length] = signal;
      phoenixSignals = newSignals;
      phoenixSignalBuses.add(busName);
      Map<String, List<BaseStatusSignal>> groups = new LinkedHashMap<>();
      for (int i = 0; i < phoenixSignals.length; i++) {
        groups
            .computeIfAbsent(phoenixSignalBuses.get(i), ignored -> new ArrayList<>())
            .add(phoenixSignals[i]);
      }
      Map<String, BaseStatusSignal[]> groupedArrays = new LinkedHashMap<>();
      groups.forEach(
          (bus, signals) -> groupedArrays.put(bus, signals.toArray(new BaseStatusSignal[0])));
      phoenixSignalsByBus = groupedArrays;
      canWaitForAll = groups.size() == 1 && new CANBus(busName).isNetworkFD();
      phoenixQueues.add(queue);
    } finally {
      Drive.odometryLock.unlock();
      signalsLock.unlock();
    }
    return queue;
  }

  /** Registers a generic signal to be read from the thread. */
  public Queue<Double> registerSignal(DoubleSupplier signal) {
    LatestSampleQueue<Double> queue = createQueue();
    signalsLock.lock();
    Drive.odometryLock.lock();
    try {
      genericSignals.add(signal);
      genericQueues.add(queue);
    } finally {
      Drive.odometryLock.unlock();
      signalsLock.unlock();
    }
    return queue;
  }

  /** Returns a new queue that returns timestamp values for each sample. */
  public Queue<Double> makeTimestampQueue() {
    LatestSampleQueue<Double> queue = createQueue();
    Drive.odometryLock.lock();
    try {
      timestampQueues.add(queue);
    } finally {
      Drive.odometryLock.unlock();
    }
    return queue;
  }

  @Override
  public void run() {
    while (true) {
      // Wait for updates from all signals
      signalsLock.lock();
      try {
        if (canWaitForAll && phoenixSignals.length > 0) {
          BaseStatusSignal.waitForAll(2.0 / SwerveConstants.kOdometryFrequency, phoenixSignals);
        } else {
          // Phoenix bulk operations cannot span CAN networks. Poll each bus separately when
          // the drivetrain is split, or when the only bus does not support FD blocking.
          Thread.sleep((long) (1000.0 / SwerveConstants.kOdometryFrequency));
          for (BaseStatusSignal[] busSignals : phoenixSignalsByBus.values()) {
            BaseStatusSignal.refreshAll(busSignals);
          }
        }
      } catch (InterruptedException e) {
        DriverStationErrors.reportWarning("Phoenix odometry thread interrupted", e.getStackTrace());
        Thread.currentThread().interrupt();
        return;
      } finally {
        signalsLock.unlock();
      }

      // Save new data to queues
      Drive.odometryLock.lock();
      try {
        // Sample timestamp is current FPGA time minus average CAN latency
        //     Default timestamps from Phoenix are NOT compatible with
        //     FPGA timestamps, this solution is imperfect but close
        double timestamp = RobotController.getTime() / 1e6;
        double totalLatency = 0.0;
        for (BaseStatusSignal signal : phoenixSignals) {
          totalLatency += signal.getTimestamp().getLatency();
        }
        if (phoenixSignals.length > 0) {
          timestamp -= totalLatency / phoenixSignals.length;
        }

        // Add new samples to queues
        for (int i = 0; i < phoenixSignals.length; i++) {
          offerSample(phoenixQueues.get(i), phoenixSignals[i].getValueAsDouble());
        }
        for (int i = 0; i < genericSignals.size(); i++) {
          offerSample(genericQueues.get(i), genericSignals.get(i).getAsDouble());
        }
        for (LatestSampleQueue<Double> timestampQueue : timestampQueues) {
          offerSample(timestampQueue, timestamp);
        }
      } finally {
        Drive.odometryLock.unlock();
      }

      // every ~1s
      if ((loopCount++ % (int) SwerveConstants.kOdometryFrequency) == 0) {
        Logger.recordOutput("Drive/OdomThread/DroppedSamples", droppedSamples);
      }
    }
  }

  private static LatestSampleQueue<Double> createQueue() {
    return new LatestSampleQueue<>(QUEUE_CAPACITY);
  }

  private void offerSample(LatestSampleQueue<Double> queue, double sample) {
    if (queue.offerLatest(sample)) {
      droppedSamples++;
    }
  }
}
