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

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.util.Alert;

public class FieldState {

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

    // The HUB is active for both alliances in AUTO
    if (DriverStation.isAutonomous()) {
      return true;
    }

    // The HUB is not active when not in AUTO or TELEOP
    if (!DriverStation.isTeleop()) {
      return false;
    }

    // Read in the current match time & get alliance
    // TODO: Since the ``DriverStation.getMatchTime()`` returns INT::
    //       Return the approximate match time. The FMS does not send an
    //       official match time to the robots, but does send an approximate
    //       match time. The value will count down the time remaining in the
    //       current period (auto or teleop). Warning: This is not an official
    //       time (so it cannot be used to dispute ref calls or guarantee that
    //       a function will trigger before the match ends).
    double timeRemaining = DriverStation.getMatchTime();
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

    // If the FMS has not provided an alliance yet, set to TRUE and kick an Alert!
    if (timeRemaining < 130.0 && wonAuto == null) {
      new Alert(
              "No HUB data from FMS!  HUB is listed as ACTIVE!  Driver BEWARE!",
              Alert.AlertType.WARNING)
          .set(true);
      return true;
    }

    if (timeRemaining >= 130.0) {
      // TRANSITION SHIFT -- Both HUBs active
      return true;

    } else if (timeRemaining >= 105.0) {
      // SHIFT 1 -- Non-winning alliance gets a first go
      return !(wonAuto == alliance);

    } else if (timeRemaining >= 80.0) {
      // SHIFT 2 -- Winning alliance gets a chance
      return (wonAuto == alliance);

    } else if (timeRemaining >= 55.0) {
      // SHIFT 3 -- Trade off
      return !(wonAuto == alliance);

    } else if (timeRemaining >= 30.0) {
      // SHIFT 4 -- Trade off again
      return (wonAuto == alliance);

    } else {
      // ENDGAME -- Both HUBs active
      return true;
    }
  }
}
