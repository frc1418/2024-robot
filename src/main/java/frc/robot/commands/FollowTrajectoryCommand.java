// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.common.Odometry;
import frc.robot.subsystems.SwerveDriveSubsystem;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** An example command that uses an example subsystem. */
public class FollowTrajectoryCommand extends SequentialCommandGroup {

  public FollowTrajectoryCommand(
        String trajectoryName,
        Odometry odometry,
        SwerveDriveSubsystem swerveDriveSubsystem, HashMap<String, Command> eventMap, PathConstraints pathConstraints) {

    PathPlannerTrajectory trajectory = PathPlanner.loadPath(trajectoryName, pathConstraints);
    PIDConstants angleConstants = new PIDConstants(0.9, 0, 0);
    PIDConstants translationConstants = new PIDConstants(0.001, 0, 0);

    SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(odometry::getPose,
      odometry::reset,
      translationConstants,
      angleConstants,
      swerveDriveSubsystem::drive,
      eventMap,
      true,
      swerveDriveSubsystem);
    
    addRequirements(swerveDriveSubsystem);

    addCommands(
        new PrintCommand(trajectoryName),
        new InstantCommand(() -> odometry.reset(trajectory.getInitialHolonomicPose())),
        autoBuilder.fullAuto(trajectory),
        new PrintCommand("DONE"),
        new InstantCommand(() -> swerveDriveSubsystem.drive(0, 0, 0))
    );
  }

  public FollowTrajectoryCommand(
        Alliance alliance,
        String trajectoryName,
        Odometry odometry,
        SwerveDriveSubsystem swerveDriveSubsystem, HashMap<String, Command> eventMap, PathConstraints pathConstraints) {

    this(
      trajectoryName.equals("leftToLeftBall") ? 
        (alliance == Alliance.Blue ?
          trajectoryName + "BLUE" : 
          trajectoryName + "RED")
        : trajectoryName,
        odometry, swerveDriveSubsystem, eventMap, pathConstraints);
  }
}
