// Copyright (c) 2026 Az-FIRST
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
//
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.util.Alert;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.RobotState;

public final class FieldState {
  private static final double TRANSITION_END_SECONDS = 130.0;
  private static final double SHIFT_DURATION_SECONDS = 25.0;
  private static final double ENDGAME_START_SECONDS = 30.0;

  private static final Alert missingHubDataAlert =
      new Alert(
          "No HUB data from FMS! HUB is listed as ACTIVE! Driver BEWARE!", Alert.AlertType.WARNING);

  private FieldState() {}

  /**
   * FOR 2026 - REBUILT, store the data from the FMS about the TeleOp shifts here
   *
   * <p>Once the FMS chooses an alliance, this value will become either 'B' or 'R' for which
   * alliance's HUB is INACTIVE first.
   *
   * <p>If this variable is 'B', then BLUE is ACTIVE during Shift 2 and Shift 4 (and RED is ACTIVE
   * during Shift 1 and Shift 3).
   *
   * <p>If this variable is 'R', then RED is ACTIVE during Shift 2 and Shift 4 (and BLUE is ACTIVE
   * during Shift 1 and Shift 3).
   *
   * <p>========== TESTING ==========
   *
   * <p>You can test your Game Specific Data code without FMS by using the Driver Station. Click on
   * the Setup tab of the Driver Station, then enter the desired test string into the Game Data text
   * field. The data will be transmitted to the robot in one of two conditions: Enable the robot in
   * Teleop mode, or when the DS reaches the End Game time in a Practice Match (times are
   * configurable on the Setup tab). It is recommended to run at least one match using the Practice
   * functionality to verify that your code works correctly in a full match flow.
   */
  public static Alliance wonAuto = null;

  /**
   * Check whether the HUB is active right now
   *
   * @return Whether the team's alliance's HUB is active right now
   */
  public static boolean isHubActive() {
    missingHubDataAlert.set(false);

    // The HUB is active for both alliances in AUTO
    if (RobotState.isAutonomous()) {
      return true;
    }

    // The HUB is not active when not in AUTO or TELEOP
    if (!RobotState.isTeleop()) {
      return false;
    }

    // Read the approximate match time and alliance. The FMS does not send an official match time to
    // robots; MatchState.getMatchTime() is approximate and should not be used to dispute ref
    // calls or guarantee that a function will trigger before the match ends.
    double timeRemaining = MatchState.getMatchTime();
    Alliance alliance = MatchState.getAlliance().orElse(Alliance.BLUE);

    // If the FMS has not provided an alliance yet, set to TRUE and kick an Alert!
    boolean isMissingHubData = timeRemaining < TRANSITION_END_SECONDS && wonAuto == null;
    missingHubDataAlert.set(isMissingHubData);
    if (isMissingHubData) {
      return true;
    }

    // Both HUBs are active during transition and endgame.
    if (timeRemaining >= TRANSITION_END_SECONDS || timeRemaining < ENDGAME_START_SECONDS) {
      return true;
    }

    // The non-winning alliance is active in odd shifts; the winning alliance is active in even
    // shifts.
    return isAllianceActiveDuringShift(timeRemaining, alliance, wonAuto);
  }

  static boolean isAllianceActiveDuringShift(
      double timeRemaining, Alliance alliance, Alliance autoWinner) {
    int shift = (int) Math.ceil((TRANSITION_END_SECONDS - timeRemaining) / SHIFT_DURATION_SECONDS);
    boolean wonAutoIsActive = shift % 2 == 0;
    return wonAutoIsActive == (autoWinner == alliance);
  }
}
