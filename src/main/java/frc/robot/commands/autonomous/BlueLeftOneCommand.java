package frc.robot.commands.autonomous;

import frc.robot.commands.FeedCommand;
import frc.robot.common.Odometry;
import frc.robot.subsystems.FeedSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.commands.FeedCommand;

import java.nio.file.Path;
import java.util.HashMap;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** An example command that uses an example subsystem. */
public class BlueLeftOneCommand extends SequentialCommandGroup {

    private String TRAJECTORY_NAME = "BlueLeft1Note";
    SwerveDriveSubsystem swerveDrive;
    FeedSubsystem feedSubsystem;

    public BlueLeftOneCommand(SwerveDriveSubsystem swerveDrive, FeedSubsystem feedSubsystem) {
      this.swerveDrive = swerveDrive;
      this.feedSubsystem = feedSubsystem;

      
      addRequirements(swerveDrive, feedSubsystem);

      addCommands(
        swerveDrive.followPath(PathPlannerPath.fromPathFile(TRAJECTORY_NAME))
      );
    }

}