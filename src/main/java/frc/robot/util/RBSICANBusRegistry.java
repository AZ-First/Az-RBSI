// Copyright (c) 2024-2026 Az-FIRST
// http://github.com/AZ-First
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.util;

import com.ctre.phoenix6.CANBus;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;

/** Centralized CAN bus singleton registry */
public final class RBSICANBusRegistry {
  private static final Map<String, CANBus> realBuses = new ConcurrentHashMap<>();
  private static final Map<String, CANBusLike> likeBuses = new ConcurrentHashMap<>();
  private static volatile boolean initialized = false;
  private static volatile boolean sim = false;

  /**
   * Initialize the REAL CANBusses
   *
   * @param busNames The list of bus names to initialize
   */
  public static void initReal(String... busNames) {
    sim = false;
    for (String name : busNames) {
      CANBus bus = realBuses.computeIfAbsent(name, CANBus::new);
      likeBuses.computeIfAbsent(name, n -> new RealCANBusAdapter(bus));
    }
    initialized = true;
  }

  /**
   * Initialize the SIM CANBusses
   *
   * @param busNames The list of bus names to initialize
   */
  public static void initSim(String... busNames) {
    sim = true;
    for (String name : busNames) {
      likeBuses.computeIfAbsent(name, SimCANBusStub::new);
    }
    initialized = true;
  }

  /**
   * Get the CANBus for Phoenix device constructors
   *
   * @param name Name of the CAN bus to get
   * @return CANBus object corresponding to the name
   */
  public static CANBus getBus(String name) {
    checkInit();
    if (sim) {
      throw new IllegalStateException("No real CANBus in SIM. Use getLike() or skip CTRE devices.");
    }
    CANBus bus = realBuses.get(name);
    if (bus == null) throwUnknown(name, realBuses.keySet());
    return bus;
  }

  /**
   * Get a CANBus-like object for health logging
   *
   * @param name Name of the bus to health log
   * @return CANBusLike object
   */
  public static CANBusLike getLike(String name) {
    checkInit();
    CANBusLike bus = likeBuses.get(name);
    if (bus == null) throwUnknown(name, likeBuses.keySet());
    return bus;
  }

  /** Check that the Registry is initialized */
  private static void checkInit() {
    if (!initialized) throw new IllegalStateException("RBSICANBusRegistry not initialized.");
  }

  /** Throw exception if the CAN bus name is not in the Registry */
  private static void throwUnknown(String name, Set<String> known) {
    throw new IllegalArgumentException("Unknown CAN bus '" + name + "'. Known: " + known);
  }

  /** Nested types for Registry Function *********************************** */

  /** CANBusLike interface */
  public interface CANBusLike {
    String getName();

    CANBus.CANBusStatus getStatus();
  }

  /** Real CAN Bus Adapter */
  static final class RealCANBusAdapter implements CANBusLike {
    private final CANBus bus;

    /** Constructor */
    RealCANBusAdapter(CANBus bus) {
      this.bus = bus;
    }

    /** Get the name of this CANBus instance */
    @Override
    public String getName() {
      return bus.getName();
    }

    /** Get the status of this CANBus instance */
    @Override
    public CANBus.CANBusStatus getStatus() {
      return bus.getStatus();
    }
  }

  /** Simulated CAN Bus Stub */
  static final class SimCANBusStub implements CANBusLike {
    private final String name;

    /** Constructor */
    SimCANBusStub(String name) {
      this.name = name;
    }

    /** Get the name of this simulated CANBus */
    @Override
    public String getName() {
      return name;
    }

    /** Get the status of this simulated CANBus */
    @Override
    public CANBus.CANBusStatus getStatus() {
      return new CANBus.CANBusStatus();
    }
  }
}
