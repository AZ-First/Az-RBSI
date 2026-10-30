// Copyright (c) 2024-2027 Az-FIRST
// http://github.com/AZ-First
// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import static org.junit.jupiter.api.Assertions.assertNotNull;

import frc.robot.util.RBSIEnum.AutoType;
import java.lang.reflect.Field;
import org.junit.jupiter.api.Test;

public class RobotContainerTest {

  @Test
  public void createChoreoRobotContainer() throws ReflectiveOperationException {
    Field autoTypeField = Constants.class.getDeclaredField("autoType");
    autoTypeField.setAccessible(true);
    AutoType previous = (AutoType) autoTypeField.get(null);
    try {
      autoTypeField.set(null, AutoType.CHOREO);
      RobotContainer container = new RobotContainer();
      assertNotNull(container.getAutonomousCommandChoreo());
    } finally {
      autoTypeField.set(null, previous);
    }
  }
}
