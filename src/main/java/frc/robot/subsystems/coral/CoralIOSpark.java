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

/** Add your docs here. */
public class CoralIOSpark implements CoralIO {

  private SparkMax manipulator;
  private SparkMaxConfig manipulatorConfig;

  private SparkFlex arm;
  private SparkFlexConfig armConfig;
  private SparkClosedLoopController armController;

  public CoralIOSpark() {

    manipulator = new SparkMax(CoralConstants.manipulatorCANID, MotorType.kBrushless);
    manipulatorConfig = new SparkMaxConfig();

    arm = new SparkFlex(CoralConstants.armCANID, MotorType.kBrushless);
    armController = arm.getClosedLoopController();
    armConfig = new SparkFlexConfig();

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
    armConfig.closedLoopRampRate(1);
    armConfig
        .absoluteEncoder
        .velocityConversionFactor(2.0 * Math.PI / 60)
        .positionConversionFactor(2.0 * Math.PI)
        .zeroOffset(CoralConstants.armZeroOffset);
    armConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .pid(CoralConstants.armP, CoralConstants.armI, CoralConstants.armD)
        .positionWrappingEnabled(false)
        .positionWrappingInputRange(CoralConstants.armMinPosition, CoralConstants.armMaxPosition)
        .outputRange(-CoralConstants.armPercentPower, CoralConstants.armPercentPower);
    arm.configure(
        armConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(CoralIOInputs inputs) {
    inputs.armAngle = arm.getAbsoluteEncoder().getPosition();
    inputs.manipulatorSpeed = manipulator.getEncoder().getVelocity();
    inputs.manipulatorCurrent = manipulator.getOutputCurrent();
  }

  @Override
  public void runManipulator(double speed) {
    manipulator.set(speed);
  }

  @Override
  public void moveArm(double angle) {
    armController.setReference(angle, ControlType.kPosition);
  }
}
