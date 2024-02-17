package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.FeedSubsystem;

public class FeedCommand extends SequentialCommandGroup{
    private FeedSubsystem feedSubsystem;

    public FeedCommand(FeedSubsystem feedSubsystem) {

        this.feedSubsystem = feedSubsystem;

        addRequirements(this.feedSubsystem);

        addCommands(
            // new WaitCommand(1.5).deadlineWith(
            //     new RunCommand(() -> this.feedSubsystem.feed(0.25), this.feedSubsystem)
            // ),
            // new WaitCommand(0.5).deadlineWith(
            //     new RunCommand(() -> this.feedSubsystem.feed(-0.2), this.feedSubsystem)
            // )
            new RunCommand(() -> this.feedSubsystem.feed(0.25), this.feedSubsystem).withTimeout(1.5),
            new RunCommand(() -> this.feedSubsystem.feed(-0.2), this.feedSubsystem).withTimeout(0.5)
        );
    }
}
