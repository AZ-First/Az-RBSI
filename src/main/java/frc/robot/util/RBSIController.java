// Copyright (c) 2024-2026 Az-FIRST
// http://github.com/AZ-First
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.Logger;

/**
 * Semantic wrapper for the driver controller.
 *
 * <p>The selected physical controller is detected once at robot startup. RobotContainer should bind
 * to driver actions from this class instead of binding directly to Xbox- or PlayStation-specific
 * button names.
 */
public abstract class RBSIController {
  private static final String PLAYSTATION_NAME_MARKER = "playstation";
  private static final String PS4_NAME_MARKER = "ps4";
  private static final String PS5_NAME_MARKER = "ps5";
  private static final String PS_NAME_MARKER = "ps";
  private static final String DUALSHOCK_NAME_MARKER = "dualshock";
  private static final String DUALSENSE_NAME_MARKER = "dualsense";
  private static final String WIRELESS_CONTROLLER_NAME = "wireless controller";

  private final int port;
  private final String controllerType;

  private RBSIController(int port, String controllerType) {
    this.port = port;
    this.controllerType = controllerType;
  }

  /** Creates a controller wrapper for the HID currently connected at startup. */
  public static RBSIController createDriverController(int port) {
    String name = DriverStation.getJoystickName(port);
    RBSIController controller = createController(port, name);

    Logger.recordOutput("DriverController/Port", port);
    Logger.recordOutput("DriverController/Name", name);
    Logger.recordOutput("DriverController/Type", controller.getControllerType());
    return controller;
  }

  private static RBSIController createController(int port, String name) {
    if (DriverStation.getJoystickIsXbox(port)) {
      return new XboxDriverController(port);
    }

    String normalizedName = name == null ? "" : name.toLowerCase();
    if (normalizedName.contains(DUALSENSE_NAME_MARKER)
        || normalizedName.contains(PS5_NAME_MARKER)
        || normalizedName.contains(WIRELESS_CONTROLLER_NAME)) {
      return new PS5DriverController(port);
    }
    if (normalizedName.contains(DUALSHOCK_NAME_MARKER)
        || normalizedName.contains(PS4_NAME_MARKER)
        || normalizedName.contains(PLAYSTATION_NAME_MARKER)
        || normalizedName.contains(PS_NAME_MARKER)) {
      return new PS4DriverController(port);
    }

    return new XboxDriverController(port);
  }

  public int getPort() {
    return port;
  }

  public String getControllerType() {
    return controllerType;
  }

  public abstract Trigger robotRelative();

  public abstract Trigger brake();

  public abstract Trigger xLock();

  public abstract Trigger zeroGyro();

  public abstract Trigger runFlywheel();

  public abstract Trigger autopilotDemo();

  public abstract Trigger povLeft();

  public abstract Trigger povRight();

  public abstract Trigger povUp();

  public abstract Trigger povDown();

  public abstract double getLeftX();

  public abstract double getLeftY();

  public abstract double getRightX();

  public abstract double getRightY();

  private static final class XboxDriverController extends RBSIController {
    private final CommandXboxController controller;

    private XboxDriverController(int port) {
      super(port, "Xbox");
      controller = new CommandXboxController(port);
    }

    @Override
    public Trigger robotRelative() {
      return controller.b();
    }

    @Override
    public Trigger brake() {
      return controller.a();
    }

    @Override
    public Trigger xLock() {
      return controller.x();
    }

    @Override
    public Trigger zeroGyro() {
      return controller.y();
    }

    @Override
    public Trigger runFlywheel() {
      return controller.rightBumper();
    }

    @Override
    public Trigger autopilotDemo() {
      return controller.leftBumper();
    }

    @Override
    public Trigger povLeft() {
      return controller.povLeft();
    }

    @Override
    public Trigger povRight() {
      return controller.povRight();
    }

    @Override
    public Trigger povUp() {
      return controller.povUp();
    }

    @Override
    public Trigger povDown() {
      return controller.povDown();
    }

    @Override
    public double getLeftX() {
      return controller.getLeftX();
    }

    @Override
    public double getLeftY() {
      return controller.getLeftY();
    }

    @Override
    public double getRightX() {
      return controller.getRightX();
    }

    @Override
    public double getRightY() {
      return controller.getRightY();
    }
  }

  private static final class PS4DriverController extends RBSIController {
    private final CommandPS4Controller controller;

    private PS4DriverController(int port) {
      super(port, "PS4");
      controller = new CommandPS4Controller(port);
    }

    @Override
    public Trigger robotRelative() {
      return controller.circle();
    }

    @Override
    public Trigger brake() {
      return controller.cross();
    }

    @Override
    public Trigger xLock() {
      return controller.square();
    }

    @Override
    public Trigger zeroGyro() {
      return controller.triangle();
    }

    @Override
    public Trigger runFlywheel() {
      return controller.R1();
    }

    @Override
    public Trigger autopilotDemo() {
      return controller.L1();
    }

    @Override
    public Trigger povLeft() {
      return controller.povLeft();
    }

    @Override
    public Trigger povRight() {
      return controller.povRight();
    }

    @Override
    public Trigger povUp() {
      return controller.povUp();
    }

    @Override
    public Trigger povDown() {
      return controller.povDown();
    }

    @Override
    public double getLeftX() {
      return controller.getLeftX();
    }

    @Override
    public double getLeftY() {
      return controller.getLeftY();
    }

    @Override
    public double getRightX() {
      return controller.getRightX();
    }

    @Override
    public double getRightY() {
      return controller.getRightY();
    }
  }

  private static final class PS5DriverController extends RBSIController {
    private final CommandPS5Controller controller;

    private PS5DriverController(int port) {
      super(port, "PS5");
      controller = new CommandPS5Controller(port);
    }

    @Override
    public Trigger robotRelative() {
      return controller.circle();
    }

    @Override
    public Trigger brake() {
      return controller.cross();
    }

    @Override
    public Trigger xLock() {
      return controller.square();
    }

    @Override
    public Trigger zeroGyro() {
      return controller.triangle();
    }

    @Override
    public Trigger runFlywheel() {
      return controller.R1();
    }

    @Override
    public Trigger autopilotDemo() {
      return controller.L1();
    }

    @Override
    public Trigger povLeft() {
      return controller.povLeft();
    }

    @Override
    public Trigger povRight() {
      return controller.povRight();
    }

    @Override
    public Trigger povUp() {
      return controller.povUp();
    }

    @Override
    public Trigger povDown() {
      return controller.povDown();
    }

    @Override
    public double getLeftX() {
      return controller.getLeftX();
    }

    @Override
    public double getLeftY() {
      return controller.getLeftY();
    }

    @Override
    public double getRightX() {
      return controller.getRightX();
    }

    @Override
    public double getRightY() {
      return controller.getRightY();
    }
  }
}
