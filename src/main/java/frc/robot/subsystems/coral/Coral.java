// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.coral;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Coral extends SubsystemBase {

  private final CoralIO io;
  private final CoralIOInputsAutoLogged inputs = new CoralIOInputsAutoLogged();

  private double armSetPosition;
  private double manipulatorSetSpeed;

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

    io.runManipulator(manipulatorSetSpeed);
    io.moveArm(armSetPosition);

    Logger.recordOutput("CoralStates/ManipulatorSetSpeed", manipulatorSetSpeed);
    Logger.recordOutput("CoralStates/ArmSetPosition", armSetPosition);

    Logger.recordOutput("CoralStates/SystemState", systemState);
    Logger.recordOutput("CoralStates/PreviousState", previousState);
    Logger.recordOutput("CoralStates/WantedState", wantedState);
  }

  /**
   * @param position position that is needed. Usually the {@code armSetPosition}
   * @return if the arm is close enough to the arm set position
   */
  public boolean isArmInPosition(double position) {
    return position + CoralConstants.armTolerance < inputs.armAngle
        || inputs.armAngle < position + CoralConstants.armTolerance;
  }

  /**
   * Updates the current state based on the wanted state. Should be used like this: {@code systemState = updateState();}
   * @return The system state
   */
  private SystemState updateState() {
    previousState = systemState;
    return switch (wantedState) {
      case IDLE:
        yield SystemState.IDLE;
      case INTAKING:
        if (inputs.hasCoral) yield SystemState.SCORING;
        yield SystemState.INTAKING;
      case SCORING:
        if (!isArmInPosition(CoralConstants.L1Position)) {
          yield SystemState.SCORING;
        }
        yield SystemState.EJECTING;
      case RESTING:
        yield SystemState.RESTING;
    };
  }

  /**
   * Sets the variables that command the interfaces to values based on {@code systemState}
   */
  private void applyState() {
    switch (systemState) {
      case IDLE:
        break;
      case INTAKING:
        armSetPosition = CoralConstants.intakePosition;
        manipulatorSetSpeed = CoralConstants.coralIntakeSpeed;
        break;
      case SCORING:
        armSetPosition = CoralConstants.L1Position;
        manipulatorSetSpeed = 0.0;
        break;
      case EJECTING:
        armSetPosition = CoralConstants.L1Position;
        manipulatorSetSpeed = CoralConstants.coralScoringSpeed;
        break;
      case RESTING:
        armSetPosition = CoralConstants.inPosition;
        manipulatorSetSpeed = 0.0;
        break;
    }
  }

  /**
   * Sets the wanted state for the coral manipulator
   * @param wantedState state that is wanted
   */
  public void setWantedState(WantedState wantedState) {
    this.wantedState = wantedState;
  }
}
