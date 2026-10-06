package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.Test;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;

class CommandsV3SchedulerTest {
  @Test
  void driveStyleOverrideInterruptsDefaultAndDefaultResumesAfterCancel() {
    Scheduler scheduler = Scheduler.createIndependentScheduler();
    Mechanism mechanism = new Mechanism("Test Drive", scheduler);
    AtomicInteger defaultCycles = new AtomicInteger();
    AtomicInteger overrideCycles = new AtomicInteger();
    AtomicInteger cancellations = new AtomicInteger();

    Command defaultCommand =
        mechanism
            .runRepeatedly(defaultCycles::incrementAndGet)
            .withPriority(Command.LOWEST_PRIORITY + 1)
            .named("Default Drive");
    Command override =
        mechanism
            .runRepeatedly(overrideCycles::incrementAndGet)
            .whenCanceled(cancellations::incrementAndGet)
            .named("Override Drive");
    mechanism.setDefaultCommand(defaultCommand);

    scheduler.run();
    assertEquals(1, defaultCycles.get());

    scheduler.schedule(override);
    scheduler.run();
    assertFalse(scheduler.isRunning(defaultCommand));
    assertTrue(scheduler.isRunning(override));
    assertEquals(1, overrideCycles.get());

    scheduler.cancel(override);
    assertEquals(1, cancellations.get());
    scheduler.run();
    assertTrue(scheduler.isRunning(defaultCommand));
    assertEquals(2, defaultCycles.get());
  }
}
