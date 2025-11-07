// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.coral.Coral;

public class Superstructure extends SubsystemBase {

  private Coral coral;

  public enum CurrentState {
    INTAKING,
    STOWING,
    SCORINGL1,
    EJECTL1,
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
      case STOWING:
        stow();
        break;
      case SCORINGL1:
        scoreL1();
        break;
      case EJECTL1:
        ejectL1();
        break;
      case INTAKING:
        intake();
      default:
        currentState = CurrentState.IDLE;
        break;
    }
  }

  private void intake() {
    coral.setWantedState(Coral.SystemState.INTAKING);
  }

  private void scoreL1() {
    coral.setWantedState(Coral.SystemState.SCORING);
  }

  private void ejectL1() {
    coral.setWantedState(Coral.SystemState.EJECTL1);
  }

  private void stow() {
    coral.setWantedState(Coral.SystemState.RESTING);
  }

  public void setWantedState(CurrentState state) {
    this.currentState = state;
  }

  public Command setStateCommand(CurrentState state) {
    return new InstantCommand(() -> setStateCommand(state));
  }
}
