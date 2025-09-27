// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.coral;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Coral extends SubsystemBase {

  private final CoralIO io;
  private final CoralIOInputsAutoLogged inputs = new CoralIOInputsAutoLogged();

  /** Creates a new Coral. */
  public Coral(CoralIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Coral", inputs);
  }

  public void moveArm(double angle) {
    io.moveArm(angle);
    Logger.recordOutput("CoralInputs/ArmAngle", angle);
  }

  public void runManipulator(double speed) {
    io.runManipulator(speed);
    Logger.recordOutput("CoralInputs/ManipulatorSpeed", speed);
  }

  public double coralManipulatorCurrent() {
    return inputs.manipulatorCurrent;
  }

  public double getArmAngle() {
    return inputs.armAngle;
  }
}
