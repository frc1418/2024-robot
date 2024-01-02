// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import frc.robot.commands.FollowTrajectoryCommand;
import frc.robot.common.Odometry;
import frc.robot.subsystems.SwerveDriveSubsystem;

import java.util.HashMap;
import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** An example command that uses an example subsystem. */
public class ChargeCommand extends SequentialCommandGroup {

    private String TRAJECTORY_NAME = "middleToChargingStation";

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ChargeCommand(SwerveDriveSubsystem swerveDriveSubsystem, Odometry odometry, HashMap<String, Command> eventMap) {
    addCommands(
      new FollowTrajectoryCommand(TRAJECTORY_NAME, odometry, swerveDriveSubsystem, eventMap, new PathConstraints(2.5, 2.5))
      );

    addRequirements(swerveDriveSubsystem);
  }
}
