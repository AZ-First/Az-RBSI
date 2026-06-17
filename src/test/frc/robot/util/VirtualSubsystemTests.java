package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class VirtualSubsystemTests {

  @BeforeEach
  void resetRegistry() {
    VirtualSubsystem.resetForTesting();
  }

  @AfterEach
  void cleanupRegistry() {
    VirtualSubsystem.resetForTesting();
  }

  @Test
  void periodicAllOrdersByPriorityThenConstructionOrder() {
    List<String> calls = new ArrayList<>();

    new RecordingSubsystem("middleA", 0, calls);
    new RecordingSubsystem("early", -30, calls);
    new RecordingSubsystem("middleB", 0, calls);
    new RecordingSubsystem("late", 20, calls);

    VirtualSubsystem.periodicAll();

    assertEquals(List.of("early", "middleA", "middleB", "late"), calls);
  }

  @Test
  void periodicAllResortsWhenSubsystemIsAddedAfterFirstRun() {
    List<String> calls = new ArrayList<>();

    new RecordingSubsystem("middleA", 0, calls);
    new RecordingSubsystem("middleB", 0, calls);

    VirtualSubsystem.periodicAll();
    calls.clear();

    new RecordingSubsystem("early", -10, calls);

    VirtualSubsystem.periodicAll();

    assertEquals(List.of("early", "middleA", "middleB"), calls);
  }

  private static final class RecordingSubsystem extends VirtualSubsystem {
    private final String name;
    private final int priority;
    private final List<String> calls;

    RecordingSubsystem(String name, int priority, List<String> calls) {
      this.name = name;
      this.priority = priority;
      this.calls = calls;
    }

    @Override
    protected int getPeriodPriority() {
      return priority;
    }

    @Override
    protected void rbsiPeriodic() {
      calls.add(name);
    }
  }
}
