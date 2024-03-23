package frc.robot.commands.autonomous;

import frc.robot.commands.FeedCommand;
import frc.robot.commands.ShootOnceCommand;
import frc.robot.common.Odometry;
import frc.robot.subsystems.FeedSubsystem;
import frc.robot.subsystems.PivotSubsystem;
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

/** An example command that uses an example subsystem. */
public class BlueLeftOneCommand extends SequentialCommandGroup {

    private String TRAJECTORY_NAME = "BlueLeft1";
    SwerveDriveSubsystem swerveDrive;
    FeedSubsystem feedSubsystem;
    ShooterSubsystem shooter;
    PivotSubsystem pivot;

    public BlueLeftOneCommand(SwerveDriveSubsystem swerveDrive, FeedSubsystem feedSubsystem, ShooterSubsystem shooter, PivotSubsystem pivot) {
      this.swerveDrive = swerveDrive;
      this.feedSubsystem = feedSubsystem;
      this.shooter = shooter;
      this.pivot = pivot;

      
      addRequirements(swerveDrive, feedSubsystem);

      addCommands(
        new ShootOnceCommand(shooter, feedSubsystem, pivot).deadlineWith(
          new RunCommand(() -> {
            swerveDrive.turtle();
          }, swerveDrive)
        )
      );
    }

}