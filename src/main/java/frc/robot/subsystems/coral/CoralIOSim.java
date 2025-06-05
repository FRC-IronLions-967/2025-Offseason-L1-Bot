// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.coral;

import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

/** Add your docs here. */
public class CoralIOSim implements CoralIO {

  // Arm
  private LoggedMechanism2d armCanvas;
  private LoggedMechanismRoot2d armJoint;
  private LoggedMechanismLigament2d arm;
  private LoggedMechanismLigament2d armBend;
  private SingleJointedArmSim armSim;

  // Manipulator
  private SparkFlexSim manipulator;
  private double speed;

  public CoralIOSim() {
    armCanvas = new LoggedMechanism2d(0.762, 0.762);
    armJoint = armCanvas.getRoot("Arm", 0.689, 0.2);
    arm = armJoint.append(new LoggedMechanismLigament2d("Arm", 0.2, 90));
    armBend = arm.append(new LoggedMechanismLigament2d("armBend", 0.25, -37));
    armSim = new SingleJointedArmSim(DCMotor.getNEO(1), CoralConstants.armMotorReduction, CoralConstants.armMOI, CoralConstants.armLength, CoralConstants.armMinPosition, CoralConstants.armMaxPosition,
     true, CoralConstants.armMinPosition, null);

    manipulator =
        new SparkFlexSim(
            new SparkFlex(CoralConstants.manipulatorCANID, MotorType.kBrushless),
            DCMotor.getNeoVortex(1));
  }

  @Override
  public void updateInputs(CoralIOInputs inputs) {
    Logger.recordOutput("Arm", armCanvas);

    // armSim.setInput(null);

    manipulator.iterate(speed, 12, Robot.defaultPeriodSecs);

    inputs.armAngle = arm.getAngle();
    inputs.manipulatorSpeed = manipulator.getRelativeEncoderSim().getVelocity();
  }

  @Override
  public void moveArm(double angle) {
    arm.setAngle(angle);
  }

  @Override
  public void runManipulator(double speed) {
    this.speed = speed;
  }
}
