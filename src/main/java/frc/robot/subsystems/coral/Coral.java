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

  public enum SystemState {
    IDLE,
    INTAKING,
    SCORING,
    EJECTL1,
    RESTING
  }

  private SystemState systemState = SystemState.IDLE;

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Coral", inputs);

    applyStates();
  }

  public void applyStates() {
    boolean wantsToEJECTL1 = false;
    switch (systemState) {
      case IDLE:
        io.moveArm(inputs.armAngle);
        io.runManipulator(0);
        break;
      case INTAKING:
        if (!inputs.hasCoral) {
          io.moveArm(CoralConstants.intakePosition);
          io.runManipulator(CoralConstants.coralIntakeSpeed);
        } else {
          systemState = SystemState.RESTING;
        }
        break;
      case SCORING:
        if (inputs.hasCoral) {
          io.moveArm(CoralConstants.L1Position);
        } else if (wantsToEJECTL1) {
          systemState = SystemState.EJECTL1;
        } else {
          systemState = SystemState.RESTING;
        }
        break;
      case EJECTL1:
        if (!inputs.hasCoral) {
          systemState = SystemState.RESTING;
        } else if (!(inputs.armAngle <= CoralConstants.armTolerance + CoralConstants.L1Position
            && inputs.armAngle <= CoralConstants.L1Position - CoralConstants.armTolerance)) {
          systemState = SystemState.SCORING;
        } else {
          io.runManipulator(CoralConstants.coralScoringSpeed);
        }
        break;
      case RESTING:
        io.moveArm(CoralConstants.inPosition);
        io.runManipulator(0);
        break;
      default:
        break;
    }
  }

  public void setWantedState(SystemState wantedState) {
    systemState = wantedState;
  }
}
