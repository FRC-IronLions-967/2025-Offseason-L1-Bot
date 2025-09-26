// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.coral;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface CoralIO {

  @AutoLog
  public static class CoralIOInputs {
    public double armAngle;
    public double manipulatorSpeed;
    public double manipulatorCurrent;
  }

  public default void updateInputs(CoralIOInputs inputs) {}

  public default void runManipulator(double speed) {}

  public default void moveArm(double angle) {}
}
