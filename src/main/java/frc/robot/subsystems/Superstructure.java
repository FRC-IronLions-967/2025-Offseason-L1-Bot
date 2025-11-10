// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.coral.Coral;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {

  private Coral coral;

  private enum CurrentState {
    INTAKING,
    STOWING,
    SCORING,
    IDLE
  }

  public enum WantedState {
    INTAKING,
    STOWED,
    SCORING,
    IDLE
  }

  private CurrentState currentState = CurrentState.IDLE;
  private WantedState wantedState = WantedState.IDLE;

  /** Creates a new Superstructure. */
  public Superstructure(Coral coral) {
    this.coral = coral;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    currentState = updateState();
    applyState();

    Logger.recordOutput("Superstructure/CurrentState", currentState);
    Logger.recordOutput("Superstructure/WantedState", wantedState);
  }

  private CurrentState updateState() {
    return switch (wantedState) {
      case INTAKING:
        yield CurrentState.INTAKING;
      case STOWED:
        yield CurrentState.STOWING;
      case SCORING:
        yield CurrentState.SCORING;
      case IDLE:
        yield CurrentState.IDLE;
      default:
        yield CurrentState.IDLE;
    };
  }

  private void applyState() {
    switch (currentState) {
      case INTAKING:
        coral.setWantedState(Coral.WantedState.INTAKING);
        break;
      case STOWING:
        coral.setWantedState(Coral.WantedState.RESTING);
        break;
      case SCORING:
        coral.setWantedState(Coral.WantedState.SCORING);
        break;
      case IDLE:
        coral.setWantedState(Coral.WantedState.IDLE);
      default:
        coral.setWantedState(Coral.WantedState.IDLE);
        break;
    }
  }

  public void setWantedState(WantedState state) {
    wantedState = state;
  }

  public Command setStateCommand(WantedState state) {
    return new InstantCommand(
        () -> {
          setWantedState(state);
        });
  }
}
