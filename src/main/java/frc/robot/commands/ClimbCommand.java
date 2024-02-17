package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.FeedSubsystem;

public class ClimbCommand extends SequentialCommandGroup{
    private ClimberSubsystem climberSubsystem;

    public ClimbCommand(ClimberSubsystem climberSubsystem) {

        this.climberSubsystem = climberSubsystem;

        addRequirements(this.climberSubsystem);

        addCommands(
            new RunCommand(() -> this.climberSubsystem.climb(0.3), this.climberSubsystem).withTimeout(2),
            new RunCommand(() -> this.climberSubsystem.climb(-0.5), this.climberSubsystem).withTimeout(2)
        );
    }
}
