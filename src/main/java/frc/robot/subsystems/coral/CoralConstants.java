// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.coral;

/** Add your docs here. */
public class CoralConstants {

  public static final double manipulatorP = 1.0;
  public static final double manipulatorI = 0.0;
  public static final double manipulatorD = 0.0;

  public static final double armZeroOffset = 0.0;

  public static final double armP = 1.0;
  public static final double armI = 0.0;
  public static final double armD = 0.0;

  public static final double armMinPosition = 0;
  public static final double armMaxPosition = Math.PI;

  public static final double coralIntakeSpeed = 5600;
  public static final double coralScoringSpeed = -5600;
  public static final boolean coralIntakeSpeedPositive = coralIntakeSpeed > 0;

  public static final double inPosition = 0.0;
  public static final double intakePosition = Math.PI;
  public static final double L1Position = 2.5;
  public static final double armTolerance = 0.1;
}
