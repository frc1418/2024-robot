package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.FeedSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootOnceCommand extends SequentialCommandGroup{
    private ShooterSubsystem shooter;
    private FeedSubsystem feeder;

    public ShootOnceCommand(ShooterSubsystem shooter, FeedSubsystem feeder) {

        this.shooter = shooter;
        this.feeder = feeder;

        addRequirements(this.shooter);

        addCommands(
            new RunCommand(()-> {
              shooter.shoot(0.75);
              feeder.feed(0.25);
            }, shooter, feeder)
        );
    }
}
