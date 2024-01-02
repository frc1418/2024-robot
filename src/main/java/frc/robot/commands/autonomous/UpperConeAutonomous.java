package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DeliverUpperConeCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;

public class UpperConeAutonomous extends SequentialCommandGroup {

    public UpperConeAutonomous(PivotSubsystem pivotSubsystem, TelescopeSubsystem telescopeSubsystem, GrabberSubsystem grabberSubsystem, ElevatorSubsystem elevatorSubsystem){

        addCommands(
            new DeliverUpperConeCommand(pivotSubsystem, telescopeSubsystem, grabberSubsystem),
            new RunCommand(() -> elevatorSubsystem.setElevatorMotor(-0.9), elevatorSubsystem)
        );
    }
    
}
