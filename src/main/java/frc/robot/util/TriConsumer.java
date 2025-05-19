// Copyright (c) 2024 FRC 6995
// https://github.com/frc6995
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

package frc.robot.util;

@FunctionalInterface
public interface TriConsumer<T, U, V> {
  public void accept(T arg0, U arg1, V arg2);
}
