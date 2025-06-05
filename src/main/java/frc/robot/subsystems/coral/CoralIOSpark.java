// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.coral;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;

/** Add your docs here. */
public class CoralIOSpark implements CoralIO {

  private SparkFlex manipulator;
  private SparkFlexConfig manipulatorConfig;
  private SparkClosedLoopController manipulatorController;

  private SparkMax arm;
  private SparkMaxConfig armConfig;
  private SparkClosedLoopController armController;

  private DigitalInput coralLimitSwitch;

  public CoralIOSpark() {

    manipulator = new SparkFlex(CoralConstants.armCANID, MotorType.kBrushless);
    manipulatorController = manipulator.getClosedLoopController();
    manipulatorConfig = new SparkFlexConfig();

    arm = new SparkMax(CoralConstants.manipulatorCANID, MotorType.kBrushless);
    armController = arm.getClosedLoopController();
    armConfig = new SparkMaxConfig();

    manipulatorConfig.smartCurrentLimit(40).idleMode(IdleMode.kCoast);
    manipulatorConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(CoralConstants.manipulatorP, CoralConstants.manipulatorI, CoralConstants.manipulatorD);

    manipulator.configure(
        armConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);

    armConfig.smartCurrentLimit(40).idleMode(IdleMode.kBrake);
    armConfig
        .absoluteEncoder
        .velocityConversionFactor(2.0 * Math.PI / 60)
        .positionConversionFactor(2.0 * Math.PI)
        .zeroOffset(CoralConstants.armZeroOffset);
    armConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .pid(CoralConstants.armP, CoralConstants.armI, CoralConstants.armD)
        .positionWrappingInputRange(CoralConstants.armMinPosition, CoralConstants.armMaxPosition)
        .positionWrappingEnabled(false);

    arm.configure(
        armConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);

    coralLimitSwitch = new DigitalInput(0);
  }

  @Override
  public void updateInputs(CoralIOInputs inputs) {
    inputs.armAngle = arm.getEncoder().getPosition();
    inputs.coralIn = coralLimitSwitch.get();
    inputs.manipulatorSpeed = manipulator.getEncoder().getVelocity();
  }

  @Override
  public void runManipulator(double speed) {
    manipulatorController.setReference(speed, ControlType.kVelocity);
  }

  @Override
  public void moveArm(double angle) {
    armController.setReference(angle, ControlType.kPosition);
  }
}
