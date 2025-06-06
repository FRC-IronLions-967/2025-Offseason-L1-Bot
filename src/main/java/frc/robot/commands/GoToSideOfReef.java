// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import java.util.List;
import org.littletonrobotics.junction.Logger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GoToSideOfReef extends Command {
  private Drive drive;
  private Pose2d pose;
  private static PathConstraints constraints;
  /** Creates a new GoToSideOfReef. */
  public GoToSideOfReef(Drive drive) {
    this.drive = drive;
    pose = new Pose2d(1000, 1000, new Rotation2d()); // something reduculous
    constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    for (Pose2d pose2d : DriveConstants.sidesOfTheReef) {
      // uses pythagorean theorm to figure closest side of the reef
      if (Math.sqrt(
              Math.pow(Math.abs(pose2d.getX() - drive.getPose().getX()), 2)
                  + Math.pow(Math.abs(pose2d.getY() - drive.getPose().getY()), 2))
          < Math.sqrt(
              Math.pow(Math.abs(pose2d.getX() - pose.getX()), 2)
                  + Math.pow(Math.abs(pose2d.getY() - pose.getY()), 2))) {
        pose = pose2d;
      }
    }
    Logger.recordOutput("SetPose", pose);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(drive.getPose(), pose);
    PathPlannerPath path =
        new PathPlannerPath(
            waypoints, constraints, null, new GoalEndState(0.0, pose.getRotation()));
    AutoBuilder.followPath(path);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the pose is close enough.
  @Override
  public boolean isFinished() {
    return Math.abs(drive.getPose().getX() - pose.getX())
            < DriveConstants.robotPoseTranslationTolerance
        && Math.abs(drive.getPose().getY() - pose.getY())
            < DriveConstants.robotPoseTranslationTolerance
        && Math.abs(drive.getPose().getRotation().getRadians() - pose.getRotation().getRadians())
            < DriveConstants.robotPoseRotationTolerance;
  }
}
