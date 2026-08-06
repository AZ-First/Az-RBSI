// Copyright (c) 2024-2026 Az-FIRST
// http://github.com/AZ-First
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.

package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class LatestSampleQueueTest {
  @Test
  void overflowEvictsOldestAndRetainsNewestInOrder() {
    LatestSampleQueue<Integer> queue = new LatestSampleQueue<>(3);

    assertFalse(queue.offerLatest(1));
    assertFalse(queue.offerLatest(2));
    assertFalse(queue.offerLatest(3));
    assertTrue(queue.offerLatest(4));

    assertEquals(3, queue.size());
    assertEquals(2, queue.poll());
    assertEquals(3, queue.poll());
    assertEquals(4, queue.poll());
    assertNull(queue.poll());
  }
}
