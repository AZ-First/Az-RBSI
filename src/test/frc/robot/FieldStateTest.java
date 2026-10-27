// Copyright (c) 2026 Az-FIRST
// http://github.com/AZ-First
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.

package frc.robot;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import org.wpilib.driverstation.Alliance;

class FieldStateTest {
  @Test
  void autoWinnerIsActiveDuringEvenShiftsIncludingBoundaries() {
    assertFalse(FieldState.isAllianceActiveDuringShift(105.0, Alliance.BLUE, Alliance.BLUE));
    assertTrue(FieldState.isAllianceActiveDuringShift(104.999, Alliance.BLUE, Alliance.BLUE));
    assertTrue(FieldState.isAllianceActiveDuringShift(80.0, Alliance.BLUE, Alliance.BLUE));
    assertFalse(FieldState.isAllianceActiveDuringShift(79.999, Alliance.BLUE, Alliance.BLUE));
    assertFalse(FieldState.isAllianceActiveDuringShift(55.0, Alliance.BLUE, Alliance.BLUE));
    assertTrue(FieldState.isAllianceActiveDuringShift(54.999, Alliance.BLUE, Alliance.BLUE));
    assertTrue(FieldState.isAllianceActiveDuringShift(30.0, Alliance.BLUE, Alliance.BLUE));
  }

  @Test
  void nonWinnerHasComplementaryShiftSchedule() {
    assertTrue(FieldState.isAllianceActiveDuringShift(105.0, Alliance.RED, Alliance.BLUE));
    assertFalse(FieldState.isAllianceActiveDuringShift(80.0, Alliance.RED, Alliance.BLUE));
    assertTrue(FieldState.isAllianceActiveDuringShift(55.0, Alliance.RED, Alliance.BLUE));
    assertFalse(FieldState.isAllianceActiveDuringShift(30.0, Alliance.RED, Alliance.BLUE));
  }
}
