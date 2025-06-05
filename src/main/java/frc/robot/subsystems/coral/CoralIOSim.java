// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.coral;

import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

/** Add your docs here. */
public class CoralIOSim implements CoralIO {

  // Arm
  private LoggedMechanism2d arm2d;
  private LoggedMechanismRoot2d armRoot;
  private LoggedMechanismLigament2d arm;
  private LoggedMechanismLigament2d armBend;

  // Manipulator
  private SparkFlexSim manipulatorSim;
  private SparkFlex manipulator;
  private FlywheelSim manipulatorFlywheelSim;
  private PIDController manipulatorPIDController;

  public CoralIOSim() {
    // arm

    // arm visualisation
    arm2d = new LoggedMechanism2d(0.762, 0.762);
    armRoot = arm2d.getRoot("Arm", 0.689, 0.2);
    arm = armRoot.append(new LoggedMechanismLigament2d("Arm", 0.2, 90));
    armBend = arm.append(new LoggedMechanismLigament2d("armBend", 0.25, -37));

    // manipulator
    manipulator = new SparkFlex(CoralConstants.manipulatorCANID, MotorType.kBrushless);
    manipulatorSim = new SparkFlexSim(manipulator, DCMotor.getNeoVortex(1));
    manipulatorFlywheelSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getNeoVortex(1), 0.00692, 1),
            DCMotor.getNeoVortex(1),
            0.01);
    manipulatorPIDController = new PIDController(1.0, 0, 0);
  }

  @Override
  public void updateInputs(CoralIOInputs inputs) {
    // arm
    Logger.recordOutput("Arm", arm2d);

    // manipulator
    manipulatorFlywheelSim.setInput(manipulator.getAppliedOutput() * 12);
    manipulatorFlywheelSim.update(Robot.defaultPeriodSecs);
    manipulatorSim.iterate(
        manipulatorFlywheelSim.getAngularVelocityRPM(), 12, Robot.defaultPeriodSecs);
    // inputs
    inputs.armAngle = arm.getAngle();
    inputs.manipulatorSpeed = manipulatorSim.getRelativeEncoderSim().getVelocity();
  }

  @Override
  public void moveArm(double angle) {
    arm.setAngle(Units.radiansToDegrees(angle));
  }

  @Override
  public void runManipulator(double speed) {
    manipulator.setVoltage(
        manipulatorPIDController.calculate(
            manipulatorSim.getRelativeEncoderSim().getVelocity(), speed));
  }
}
