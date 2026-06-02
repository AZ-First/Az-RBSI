// Copyright (c) 2024-2026 Az-FIRST
// http://github.com/AZ-First
// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the AdvantageKit-License.md file
// at the root directory of this project.

package frc.robot.util;

import frc.robot.Constants;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * Replay-safe tunable number backed by AdvantageKit NetworkTables inputs.
 *
 * <p>When tuning mode is enabled, the live value is read from {@code /Tuning/...}. Otherwise, the
 * default value is returned.
 */
public class LoggedTunableNumber implements DoubleSupplier {
  private static final String tableKey = "/Tuning";

  private final String key;
  private boolean hasDefault = false;
  private double defaultValue;
  private LoggedNetworkNumber dashboardNumber;
  private final Map<Integer, Double> lastHasChangedValues = new HashMap<>();

  /**
   * Create a new LoggedTunableNumber.
   *
   * @param dashboardKey Key under /Tuning
   */
  public LoggedTunableNumber(String dashboardKey) {
    this.key = tableKey + "/" + dashboardKey;
  }

  /**
   * Create a new LoggedTunableNumber with a default value.
   *
   * @param dashboardKey Key under /Tuning
   * @param defaultValue Default value
   */
  public LoggedTunableNumber(String dashboardKey, double defaultValue) {
    this(dashboardKey);
    initDefault(defaultValue);
  }

  /**
   * Set the default value. Can only be set once.
   *
   * @param defaultValue The default value
   */
  public void initDefault(double defaultValue) {
    if (hasDefault) {
      return;
    }

    hasDefault = true;
    this.defaultValue = defaultValue;

    if (Constants.tuningMode) {
      dashboardNumber = new LoggedNetworkNumber(key, defaultValue);
    }
  }

  /**
   * Get the current value.
   *
   * @return Live tuning value in tuning mode, otherwise the default
   */
  public double get() {
    if (!hasDefault) {
      return 0.0;
    }

    if (Constants.tuningMode && dashboardNumber != null) {
      return dashboardNumber.get();
    }

    return defaultValue;
  }

  /**
   * Returns true when this value has changed since the last check by the given caller.
   *
   * @param id Unique identifier for the caller, often hashCode()
   */
  public boolean hasChanged(int id) {
    double currentValue = get();
    Double lastValue = lastHasChangedValues.get(id);

    if (lastValue == null || Double.compare(currentValue, lastValue) != 0) {
      lastHasChangedValues.put(id, currentValue);
      return true;
    }

    return false;
  }

  /**
   * Runs an action if any of the tunable numbers changed.
   *
   * @param id Unique identifier for the caller, often hashCode()
   * @param action Callback with values in the same order as provided
   * @param tunableNumbers Tunable numbers to watch
   */
  public static void ifChanged(
      int id, Consumer<double[]> action, LoggedTunableNumber... tunableNumbers) {
    if (Arrays.stream(tunableNumbers).anyMatch(t -> t.hasChanged(id))) {
      action.accept(Arrays.stream(tunableNumbers).mapToDouble(LoggedTunableNumber::get).toArray());
    }
  }

  /** Runs an action if any of the tunable numbers changed. */
  public static void ifChanged(int id, Runnable action, LoggedTunableNumber... tunableNumbers) {
    ifChanged(id, values -> action.run(), tunableNumbers);
  }

  @Override
  public double getAsDouble() {
    return get();
  }
}
