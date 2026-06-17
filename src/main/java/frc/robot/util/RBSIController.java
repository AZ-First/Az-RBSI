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
import frc.robot.Constants.ControllerButtonConstants;
import org.littletonrobotics.junction.Logger;

/**
 * Controller-agnostic wrapper for the driver and operator controllers.
 *
 * <p>The selected physical controller is detected once at robot startup. RobotContainer should bind
 * to typed physical-position inputs from this class instead of binding directly to Xbox-, PS4-, or
 * PS5-specific button names. Teams can remap robot actions in {@link
 * frc.robot.Constants.ControllerButtonConstants} without editing this class.
 */
public abstract class RBSIController {
  public enum Button {
    SOUTH_FACE,
    EAST_FACE,
    WEST_FACE,
    NORTH_FACE,
    LEFT_BUMPER,
    RIGHT_BUMPER,
    LEFT_STICK,
    RIGHT_STICK,
    POV_LEFT,
    POV_RIGHT,
    POV_UP,
    POV_DOWN
  }

  public enum Axis {
    LEFT_TRIGGER,
    RIGHT_TRIGGER
  }

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

    Logger.recordOutput("Controller/Port" + port + "/Name", name);
    Logger.recordOutput("Controller/Port" + port + "/Type", controller.getControllerType());
    return controller;
  }

  private static RBSIController createController(int port, String name) {
    if (DriverStation.getJoystickIsXbox(port)) {
      return new XboxControllerAdapter(port);
    }

    String normalizedName = name == null ? "" : name.toLowerCase();
    if (normalizedName.contains(DUALSENSE_NAME_MARKER)
        || normalizedName.contains(PS5_NAME_MARKER)
        || normalizedName.contains(WIRELESS_CONTROLLER_NAME)) {
      return new PS5ControllerAdapter(port);
    }
    if (normalizedName.contains(DUALSHOCK_NAME_MARKER)
        || normalizedName.contains(PS4_NAME_MARKER)
        || normalizedName.contains(PLAYSTATION_NAME_MARKER)
        || normalizedName.contains(PS_NAME_MARKER)) {
      return new PS4ControllerAdapter(port);
    }

    return new XboxControllerAdapter(port);
  }

  public int getPort() {
    return port;
  }

  public String getControllerType() {
    return controllerType;
  }

  public abstract Trigger button(Button button);

  public abstract double axis(Axis axis);

  public Trigger axisTrigger(Axis axis) {
    return axisTrigger(axis, ControllerButtonConstants.kTriggerPressedThreshold);
  }

  public Trigger axisTrigger(Axis axis, double threshold) {
    return new Trigger(() -> axis(axis) > threshold);
  }

  public abstract double getLeftX();

  public abstract double getLeftY();

  public abstract double getRightX();

  public abstract double getRightY();

  private static final class XboxControllerAdapter extends RBSIController {
    private final CommandXboxController controller;

    private XboxControllerAdapter(int port) {
      super(port, "Xbox");
      controller = new CommandXboxController(port);
    }

    @Override
    public Trigger button(Button button) {
      return switch (button) {
        case SOUTH_FACE -> controller.a();
        case EAST_FACE -> controller.b();
        case WEST_FACE -> controller.x();
        case NORTH_FACE -> controller.y();
        case LEFT_BUMPER -> controller.leftBumper();
        case RIGHT_BUMPER -> controller.rightBumper();
        case LEFT_STICK -> controller.leftStick();
        case RIGHT_STICK -> controller.rightStick();
        case POV_LEFT -> controller.povLeft();
        case POV_RIGHT -> controller.povRight();
        case POV_UP -> controller.povUp();
        case POV_DOWN -> controller.povDown();
      };
    }

    @Override
    public double axis(Axis axis) {
      return switch (axis) {
        case LEFT_TRIGGER -> controller.getLeftTriggerAxis();
        case RIGHT_TRIGGER -> controller.getRightTriggerAxis();
      };
    }

    @Override
    public Trigger axisTrigger(Axis axis, double threshold) {
      return switch (axis) {
        case LEFT_TRIGGER -> controller.leftTrigger(threshold);
        case RIGHT_TRIGGER -> controller.rightTrigger(threshold);
      };
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

  private static final class PS4ControllerAdapter extends RBSIController {
    private final CommandPS4Controller controller;

    private PS4ControllerAdapter(int port) {
      super(port, "PS4");
      controller = new CommandPS4Controller(port);
    }

    @Override
    public Trigger button(Button button) {
      return switch (button) {
        case SOUTH_FACE -> controller.cross();
        case EAST_FACE -> controller.circle();
        case WEST_FACE -> controller.square();
        case NORTH_FACE -> controller.triangle();
        case LEFT_BUMPER -> controller.L1();
        case RIGHT_BUMPER -> controller.R1();
        case LEFT_STICK -> controller.L3();
        case RIGHT_STICK -> controller.R3();
        case POV_LEFT -> controller.povLeft();
        case POV_RIGHT -> controller.povRight();
        case POV_UP -> controller.povUp();
        case POV_DOWN -> controller.povDown();
      };
    }

    @Override
    public double axis(Axis axis) {
      return switch (axis) {
        case LEFT_TRIGGER -> controller.getL2Axis();
        case RIGHT_TRIGGER -> controller.getR2Axis();
      };
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

  private static final class PS5ControllerAdapter extends RBSIController {
    private final CommandPS5Controller controller;

    private PS5ControllerAdapter(int port) {
      super(port, "PS5");
      controller = new CommandPS5Controller(port);
    }

    @Override
    public Trigger button(Button button) {
      return switch (button) {
        case SOUTH_FACE -> controller.cross();
        case EAST_FACE -> controller.circle();
        case WEST_FACE -> controller.square();
        case NORTH_FACE -> controller.triangle();
        case LEFT_BUMPER -> controller.L1();
        case RIGHT_BUMPER -> controller.R1();
        case LEFT_STICK -> controller.L3();
        case RIGHT_STICK -> controller.R3();
        case POV_LEFT -> controller.povLeft();
        case POV_RIGHT -> controller.povRight();
        case POV_UP -> controller.povUp();
        case POV_DOWN -> controller.povDown();
      };
    }

    @Override
    public double axis(Axis axis) {
      return switch (axis) {
        case LEFT_TRIGGER -> controller.getL2Axis();
        case RIGHT_TRIGGER -> controller.getR2Axis();
      };
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
