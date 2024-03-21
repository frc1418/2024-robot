package frc.robot.commands.autonomous;

import frc.robot.commands.FeedCommand;
import frc.robot.common.Odometry;
import frc.robot.subsystems.FeedSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.commands.FeedCommand;

import java.nio.file.Path;
import java.util.HashMap;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/** An example command that uses an example subsystem. */
public class MiddleTwoNoteCommand extends SequentialCommandGroup {

    private String TRAJECTORY_NAME = "Middle2Note";
    SwerveDriveSubsystem swerveDrive;
    FeedSubsystem feedSubsystem;
    ShooterSubsystem shooter;

    public MiddleTwoNoteCommand(SwerveDriveSubsystem swerveDrive, FeedSubsystem feedSubsystem, ShooterSubsystem shooter) {
      this.swerveDrive = swerveDrive;
      this.feedSubsystem = feedSubsystem;
      this.shooter = shooter;

      addRequirements(swerveDrive, feedSubsystem);

      addCommands(
        swerveDrive.followPath(PathPlannerPath.fromPathFile(TRAJECTORY_NAME)),
        new RunCommand(() -> {
          feedSubsystem.feed(-0.1);
        }, feedSubsystem).withTimeout(0.5),
        new RunCommand(() -> {
          feedSubsystem.feed(0);
        }, feedSubsystem).withTimeout(1),        
        new RunCommand(() -> {
          feedSubsystem.feed(0.25);
          shooter.shoot(0.75);
        }, feedSubsystem, shooter).withTimeout(2)

      );
    }

}