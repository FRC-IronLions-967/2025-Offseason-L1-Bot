// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.coral;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

/** Add your docs here. */
public class CoralIOSim implements CoralIO {

  private LoggedMechanism2d armCanvas;
  private LoggedMechanismRoot2d armJoint;
  private LoggedMechanismLigament2d arm;

  public CoralIOSim() {
    armCanvas = new LoggedMechanism2d(6, 18);
    armJoint = armCanvas.getRoot("Arm", 1, 1);
    arm = armJoint.append(new LoggedMechanismLigament2d("Arm", 1, 90));
  }

  @Override
  public void updateInputs(CoralIOInputs inputs) {
    inputs.armAngle = arm.getAngle();
    Logger.recordOutput("Arm", armCanvas);
  }

  @Override
  public void moveArm(double angle) {
    arm.setAngle(angle * 360 / (2 * Math.PI));
  }
}
