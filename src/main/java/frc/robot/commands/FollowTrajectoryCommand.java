// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.common.Odometry;
import frc.robot.subsystems.SwerveDriveSubsystem;

import java.util.HashMap;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** An example command that uses an example subsystem. */
public class FollowTrajectoryCommand extends SubsystemBase {
  String trajectoryName;
  Odometry odometry;
  SwerveDriveSubsystem swerveDrive;
  Pose2d startPos;

  public FollowTrajectoryCommand(String trajectoryName, Odometry odometry, SwerveDriveSubsystem swerveDrive, Pose2d startPos)
  {
    this.trajectoryName = trajectoryName;
    this.odometry = odometry;
    this.swerveDrive = swerveDrive;
    this.startPos = startPos;

    AutoBuilder.configureHolonomic(
            this.odometry::getPose, // Robot pose supplier
            this.odometry::reset, // Method to reset odometry (will be called if your auto has a starting pose)
            this.swerveDrive::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this.swerveDrive::drive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                    4.5, // Max module speed, in m/s
                    0.969867, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
  }
}
