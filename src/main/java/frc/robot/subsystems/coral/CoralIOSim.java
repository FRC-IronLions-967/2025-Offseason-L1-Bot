// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.coral;

import com.revrobotics.sim.SparkMaxSim;
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
public class CoralIOSim extends CoralIOSpark {
  // Arm Visual
  private LoggedMechanism2d arm2d;
  private LoggedMechanismRoot2d armRoot;
  private LoggedMechanismLigament2d armStraight;
  private LoggedMechanismLigament2d armBend;

  // Manipulator
  private SparkMaxSim manipulatorMaxSim;
  private FlywheelSim manipulatorSim;
  private PIDController manipulatorPIDController;

  public CoralIOSim() {
    super();
    // arm visualisation
    arm2d = new LoggedMechanism2d(0.762, 0.762);
    armRoot = arm2d.getRoot("Arm", 0.689, 0.2);
    armStraight = armRoot.append(new LoggedMechanismLigament2d("Arm", 0.2, 90));
    armBend = armStraight.append(new LoggedMechanismLigament2d("armBend", 0.25, -37));

    // manipulator
    manipulatorMaxSim = new SparkMaxSim(manipulator, DCMotor.getNEO(1));
    manipulatorSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getNeoVortex(1), 0.00692, 1),
            DCMotor.getNeoVortex(1),
            0.01);
    manipulatorPIDController =
        new PIDController(
            CoralConstants.manipulatorP, CoralConstants.manipulatorI, CoralConstants.manipulatorD);
  }

  @Override
  public void updateInputs(CoralIOInputs inputs) {
    super.updateInputs(inputs);
    // arm visualisation
    Logger.recordOutput("Arm", arm2d);

    // manipulator
    manipulatorSim.setInput(manipulator.getAppliedOutput() * 12);
    manipulatorSim.update(Robot.defaultPeriodSecs);
    manipulatorMaxSim.iterate(manipulatorSim.getAngularVelocityRPM(), 12, Robot.defaultPeriodSecs);
    // inputs
    inputs.armAngle = Units.degreesToRadians(armStraight.getAngle());
    inputs.manipulatorSpeed = manipulatorMaxSim.getRelativeEncoderSim().getVelocity();
  }

  @Override
  public void moveArm(double angle) {
    super.moveArm(angle);
    armStraight.setAngle(Units.radiansToDegrees(angle));
  }

  @Override
  public void runManipulator(double speed) {
    super.runManipulator(speed);
    manipulator.setVoltage(
        manipulatorPIDController.calculate(
            manipulatorMaxSim.getRelativeEncoderSim().getVelocity(), speed));
  }
}
