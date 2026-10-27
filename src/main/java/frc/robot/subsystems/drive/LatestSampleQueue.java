// Copyright (c) 2024-2026 Az-FIRST
// http://github.com/AZ-First
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.

package frc.robot.subsystems.drive;

import java.util.AbstractQueue;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.concurrent.locks.ReentrantLock;

/** Fixed-capacity thread-safe queue that evicts its oldest entry when full. */
final class LatestSampleQueue<E> extends AbstractQueue<E> {
  private final int capacity;
  private final ArrayDeque<E> values;
  private final ReentrantLock lock = new ReentrantLock();

  LatestSampleQueue(int capacity) {
    if (capacity <= 0) {
      throw new IllegalArgumentException("capacity must be positive");
    }
    this.capacity = capacity;
    values = new ArrayDeque<>(capacity);
  }

  /** Adds a value and returns whether the oldest value had to be evicted. */
  boolean offerLatest(E value) {
    if (value == null) {
      throw new NullPointerException("LatestSampleQueue does not accept null values");
    }
    lock.lock();
    try {
      boolean evicted = values.size() == capacity;
      if (evicted) {
        values.removeFirst();
      }
      values.addLast(value);
      return evicted;
    } finally {
      lock.unlock();
    }
  }

  @Override
  public boolean offer(E value) {
    offerLatest(value);
    return true;
  }

  @Override
  public E poll() {
    lock.lock();
    try {
      return values.pollFirst();
    } finally {
      lock.unlock();
    }
  }

  @Override
  public E peek() {
    lock.lock();
    try {
      return values.peekFirst();
    } finally {
      lock.unlock();
    }
  }

  @Override
  public int size() {
    lock.lock();
    try {
      return values.size();
    } finally {
      lock.unlock();
    }
  }

  @Override
  public Iterator<E> iterator() {
    lock.lock();
    try {
      return new ArrayList<>(values).iterator();
    } finally {
      lock.unlock();
    }
  }
}
