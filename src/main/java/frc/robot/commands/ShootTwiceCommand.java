package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.FeedSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootTwiceCommand extends SequentialCommandGroup{
    private ShooterSubsystem shooter;
    private FeedSubsystem feeder;
    private IntakeSubsystem intake;

    public ShootTwiceCommand(ShooterSubsystem shooter, FeedSubsystem feeder, IntakeSubsystem intake) {

        this.shooter = shooter;
        this.feeder = feeder;
        this.intake = intake;

        addRequirements(this.shooter, this.feeder, this.intake);

        addCommands(
            new RunCommand(()-> {
              feeder.feed(0.15);
              intake.intake(0.5);
              shooter.shoot(-0.05);
            }, feeder, intake, shooter).withTimeout(7),
            new RunCommand(()-> {       
              feeder.feed(0.25);       
              shooter.shoot(0.75);
            },feeder, shooter).withTimeout(2)
          );
    }
}
