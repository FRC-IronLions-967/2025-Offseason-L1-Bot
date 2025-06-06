// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class DriveConstants {
  public static final double maxSpeedMetersPerSec = 4.2;
  public static final double odometryFrequency = 100.0; // Hz
  public static final double trackWidth = Units.inchesToMeters(24);
  public static final double wheelBase = Units.inchesToMeters(24);
  public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
  public static final Translation2d[] moduleTranslations =
      new Translation2d[] {
        new Translation2d(trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)
      };

  // Zeroed rotation values for each module, see setup instructions
  public static final Rotation2d frontLeftZeroRotation = new Rotation2d(-0.0105);
  public static final Rotation2d frontRightZeroRotation = new Rotation2d(-0.0143);
  public static final Rotation2d backLeftZeroRotation = new Rotation2d(0.00306);
  public static final Rotation2d backRightZeroRotation = new Rotation2d(-0.0354);

  // Device CAN IDs
  public static final int pigeonCanId = 9;

  public static final int frontLeftDriveCanId = 5;
  public static final int backLeftDriveCanId = 7;
  public static final int frontRightDriveCanId = 3;
  public static final int backRightDriveCanId = 1;

  public static final int frontLeftTurnCanId = 6;
  public static final int backLeftTurnCanId = 8;
  public static final int frontRightTurnCanId = 4;
  public static final int backRightTurnCanId = 2;

  // Drive motor configuration
  public static final int driveMotorCurrentLimit = 48;
  public static final double wheelRadiusMeters = Units.inchesToMeters(1.91);
  public static final double driveMotorReduction = 5.9;
  public static final DCMotor driveGearbox = DCMotor.getNeoVortex(1);

  // Drive encoder configuration
  public static final double driveEncoderPositionFactor =
      2 * Math.PI / driveMotorReduction; // Rotor Rotations -> Wheel Radians
  public static final double driveEncoderVelocityFactor =
      (2 * Math.PI) / 60.0 / driveMotorReduction; // Rotor RPM -> Wheel Rad/Sec

  // Drive PID configuration
  public static final double driveKp = 0.01;
  public static final double driveKd = 0.005;
  public static final double driveKs = 0.13854;
  public static final double driveKv = 0.12326;
  public static final double driveSimP = 0.05;
  public static final double driveSimD = 0.0;
  public static final double driveSimKs = 0.0;
  public static final double driveSimKv = 0.0789;

  // Turn motor configuration
  public static final boolean turnInverted = true;
  public static final int turnMotorCurrentLimit = 20;
  public static final double turnMotorReduction = 18.75;
  public static final DCMotor turnGearbox = DCMotor.getNeo550(1);

  // Turn encoder configuration
  public static final boolean turnEncoderInverted = true;
  public static final double turnEncoderPositionFactor = 2 * Math.PI; // Rotations -> Radians
  public static final double turnEncoderVelocityFactor = (2 * Math.PI) / 60.0; // RPM -> Rad/Sec

  // Turn PID configuration
  public static final double turnKp = 1.0;
  public static final double turnKd = 0.0;
  public static final double turnSimP = 8.0;
  public static final double turnSimD = 0.0;
  public static final double turnPIDMinInput = 0; // Radians
  public static final double turnPIDMaxInput = 2 * Math.PI; // Radians

  // PathPlanner configuration
  public static final double robotMassKg = 74.088;
  public static final double robotMOI = 6.883;
  public static final double wheelCOF = 1.2;
  public static final RobotConfig ppConfig =
      new RobotConfig(
          robotMassKg,
          robotMOI,
          new ModuleConfig(
              wheelRadiusMeters,
              maxSpeedMetersPerSec,
              wheelCOF,
              driveGearbox.withReduction(driveMotorReduction),
              driveMotorCurrentLimit,
              1),
          moduleTranslations);

  public static final double robotPoseTranslationTolerance = 0.5;
  public static final double robotPoseRotationTolerance = 0.1;

  public static final Pose2d[] sidesOfTheReef = {
    new Pose2d(3.82, 5.122, new Rotation2d(Units.degreesToRadians(-60))),
    new Pose2d(3.223, 4.035, new Rotation2d(Units.degreesToRadians(0))),
    new Pose2d(3.839, 2.919, new Rotation2d(Units.degreesToRadians(60))),
    new Pose2d(5.09, 2.909, new Rotation2d(Units.degreesToRadians(120))),
    new Pose2d(5.75, 4.025, new Rotation2d(Units.degreesToRadians(180))),
    new Pose2d(5.148, 5.131, new Rotation2d(Units.degreesToRadians(-120)))
  };
}
