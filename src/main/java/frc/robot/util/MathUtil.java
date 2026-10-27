// Copyright (c) 2024-2026 Az-FIRST
// http://github.com/AZ-First
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.

package frc.robot.util;

/** Small compatibility subset of the WPILib 2026 MathUtil helpers. */
public final class MathUtil {
  private MathUtil() {}

  public static double clamp(double value, double low, double high) {
    return Math.max(low, Math.min(value, high));
  }

  public static double interpolate(double startValue, double endValue, double t) {
    return startValue + (endValue - startValue) * clamp(t, 0.0, 1.0);
  }

  public static double applyDeadband(double value, double deadband) {
    return applyDeadband(value, deadband, 1.0);
  }

  public static double applyDeadband(double value, double deadband, double maxMagnitude) {
    if (Math.abs(value) <= deadband) {
      return 0.0;
    }

    if (maxMagnitude / deadband > 1.0e12) {
      return value > 0.0 ? value - deadband : value + deadband;
    }

    if (value > 0.0) {
      return maxMagnitude * (value - deadband) / (maxMagnitude - deadband);
    } else {
      return maxMagnitude * (value + deadband) / (maxMagnitude - deadband);
    }
  }

  public static double inputModulus(double input, double minimumInput, double maximumInput) {
    double modulus = maximumInput - minimumInput;

    int numMax = (int) ((input - minimumInput) / modulus);
    input -= numMax * modulus;

    int numMin = (int) ((input - maximumInput) / modulus);
    input -= numMin * modulus;

    return input;
  }

  public static double angleModulus(double angleRadians) {
    return inputModulus(angleRadians, -Math.PI, Math.PI);
  }
}
