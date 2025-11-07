// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.coral.Coral;
import frc.robot.subsystems.coral.CoralConstants;

public class Superstructure extends SubsystemBase {

  private Coral coral;

  private enum CurrentState {
    INTAKING,
    STOWING,
    SCORING,
    IDLE
  }

  private CurrentState currentState = CurrentState.IDLE;

  /** Creates a new Superstructure. */
  public Superstructure(Coral coral) {
    this.coral = coral;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    applyStates();
  }

  private void applyStates() {
    switch (currentState) {
      case IDLE:
        break;
   
      default:
        break;
    }
  }

  private void intake() {
    coral.moveArm(CoralConstants.intakePosition);
    
  }


}
