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

  private enum SystemState {
    IDLE,
    INTAKING,
    SCORING,
    EJECTING,
    RESTING
  }

  public enum WantedState {
    IDLE,
    INTAKING,
    SCORING,
    RESTING
  }

  private SystemState systemState = SystemState.IDLE;
  private SystemState previousState = SystemState.IDLE;
  private WantedState wantedState = WantedState.IDLE;

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Coral", inputs);

    systemState = updateState();
    applyState();

    Logger.recordOutput("Coral/SystemState", systemState);
    Logger.recordOutput("Coral/PreviousState", previousState);
    Logger.recordOutput("Coral/WantedState", wantedState);
  }

  private SystemState updateState() {
    previousState = systemState;
    return switch (wantedState) {
      case IDLE:
        yield SystemState.IDLE;
      case INTAKING:
        if (inputs.hasCoral) yield SystemState.SCORING;
        yield SystemState.INTAKING;
      case SCORING:
        if (!inputs.armInPosition) {
          yield SystemState.SCORING;
        }
        yield SystemState.EJECTING;
      case RESTING:
        yield SystemState.RESTING;
      default:
        yield SystemState.IDLE;
    };
  }

  public void applyState() {
    switch (systemState) {
      case IDLE:
        break;
      case INTAKING:
        io.runManipulator(CoralConstants.coralIntakeSpeed);
        io.moveArm(CoralConstants.intakePosition);
      case SCORING:
        io.moveArm(CoralConstants.L1Position);
        io.runManipulator(0);
      case EJECTING:
        io.runManipulator(CoralConstants.coralScoringSpeed);
      case RESTING:
        io.runManipulator(0);
        io.moveArm(CoralConstants.inPosition);
      default:
        systemState = SystemState.IDLE;
        break;
    }
  }

  public void setWantedState(WantedState wantedState) {
    this.wantedState = wantedState;
  }
}
