package frc.robot.commands.autonomous;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DeliverUpperConeCommand;
import frc.robot.commands.FollowTrajectoryCommand;
import frc.robot.common.Odometry;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;

public class LeftUpperConeAutonomous extends SequentialCommandGroup {

    GrabberSubsystem grabberSubsystem;
    SwerveDriveSubsystem swerveDriveSubsystem;
    Odometry odometry;
    PivotSubsystem pivotSubsystem;
    TelescopeSubsystem telescopeSubsystem;

    public LeftUpperConeAutonomous(Alliance alliance, GrabberSubsystem grabberSubsystem, PivotSubsystem pivotSubsystem,
            TelescopeSubsystem telescopeSubsystem, SwerveDriveSubsystem swerveDriveSubsystem, Odometry odometry, HashMap<String, Command> eventMap) {

        this.grabberSubsystem = grabberSubsystem;
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        this.odometry = odometry;
        this.pivotSubsystem = pivotSubsystem;
        this.telescopeSubsystem = telescopeSubsystem;

        addRequirements(grabberSubsystem, telescopeSubsystem, pivotSubsystem, swerveDriveSubsystem);

        addCommands(
            new DeliverUpperConeCommand(pivotSubsystem, telescopeSubsystem, grabberSubsystem),
            new FollowTrajectoryCommand(alliance, "leftToLeftBall", odometry, swerveDriveSubsystem, eventMap, new PathConstraints(2.5, 2.5)),
            new InstantCommand(() -> swerveDriveSubsystem.drive(0, 0, 0)),
            new InstantCommand(() -> grabberSubsystem.open()),
            new WaitCommand(0.5),
            new RunCommand(() -> telescopeSubsystem.setTelescopePosition(0.03))
        );
    }
    
}
