// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.coral;

import edu.wpi.first.math.util.Units;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

/** Add your docs here. */
public class CoralIOSim implements CoralIO {
  // Arm Visual
  private LoggedMechanism2d arm2d;
  private LoggedMechanismRoot2d armRoot;
  private LoggedMechanismLigament2d armStraight;
  private LoggedMechanismLigament2d armBend;

  private double manipulatorSpeed;

  public CoralIOSim() {
    // arm visualisation
    arm2d = new LoggedMechanism2d(0.762, 0.762);
    armRoot = arm2d.getRoot("Arm", 0.689, 0.2);
    armStraight = armRoot.append(new LoggedMechanismLigament2d("Arm", 0.2, 90));
    armBend = armStraight.append(new LoggedMechanismLigament2d("armBend", 0.25, -37));
  }

  @Override
  public void updateInputs(CoralIOInputs inputs) {
    inputs.armAngle = Units.degreesToRadians(armStraight.getAngle());
    inputs.manipulatorSpeed = manipulatorSpeed;
    // arm visualisation
    Logger.recordOutput("Arm", arm2d);
  }

  @Override
  public void moveArm(double angle) {
    armStraight.setAngle(Units.radiansToDegrees(angle));
  }

  @Override
  public void runManipulator(double speed) {
    manipulatorSpeed = speed;
  }
}
