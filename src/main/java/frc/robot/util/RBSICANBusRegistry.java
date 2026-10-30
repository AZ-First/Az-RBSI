// Copyright (c) 2024-2027 Az-FIRST
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
import java.util.concurrent.ConcurrentHashMap;

/** Centralized CAN bus singleton registry for Phoenix device constructors. */
public final class RBSICANBusRegistry {
  private static final Map<String, CANBus> realBuses = new ConcurrentHashMap<>();
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
      realBuses.computeIfAbsent(name, CANBus::new);
    }
    initialized = true;
  }

  /** Initialize the registry for simulation or replay. */
  public static void initSim() {
    sim = true;
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
      throw new IllegalStateException("No real CANBus in SIM. Skip CTRE devices.");
    }
    return getRegisteredBus(name, realBuses);
  }

  /** Check that the Registry is initialized */
  private static void checkInit() {
    if (!initialized) throw new IllegalStateException("RBSICANBusRegistry not initialized.");
  }

  private static <T> T getRegisteredBus(String name, Map<String, T> buses) {
    T bus = buses.get(name);
    if (bus == null) {
      throw new IllegalArgumentException(
          "Unknown CAN bus '" + name + "'. Known: " + buses.keySet());
    }
    return bus;
  }
}
