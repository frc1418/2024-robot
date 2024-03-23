package frc.robot.commands.autonomous;

import frc.robot.subsystems.FeedSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** An example command that uses an example subsystem. */
public class ChargeCommand extends SequentialCommandGroup {

  //SET UP WITH SHOOTER TOWARDS US

    private String TRAJECTORY_NAME = "Charge";
    SwerveDriveSubsystem swerveDrive;
    FeedSubsystem feedSubsystem;

    public ChargeCommand(SwerveDriveSubsystem swerveDrive, FeedSubsystem feedSubsystem) {
      this.swerveDrive = swerveDrive;
      this.feedSubsystem = feedSubsystem;

      
      addRequirements(swerveDrive, feedSubsystem);

      addCommands(
        swerveDrive.followPath(PathPlannerPath.fromPathFile(TRAJECTORY_NAME))
      );
    }

}