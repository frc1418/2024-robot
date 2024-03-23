package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.FeedSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class IntakeNoteCommand extends SequentialCommandGroup{
    private IntakeSubsystem intake;
    private FeedSubsystem feeder;

    public IntakeNoteCommand(IntakeSubsystem intake, FeedSubsystem feeder) {

        this.intake = intake;
        this.feeder = feeder;

        addRequirements(this.intake);

        addCommands(
            new RunCommand(()-> {
              intake.intake(0.5);
              feeder.feed(0.25);
            }, intake, feeder).withTimeout(4)
        );
    }
}
