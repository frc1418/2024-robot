package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;

public class DeliverMiddleConeCommand extends SequentialCommandGroup {

    private PivotSubsystem pivotSubsystem;
    private TelescopeSubsystem telescopeSubsystem;
    private GrabberSubsystem grabberSubsystem;

    private boolean droppedCone;

    public DeliverMiddleConeCommand(PivotSubsystem pivotSubsystem, TelescopeSubsystem telescopeSubsystem, GrabberSubsystem grabberSubsystem){
        this.pivotSubsystem = pivotSubsystem;
        this.telescopeSubsystem = telescopeSubsystem;
        this.grabberSubsystem = grabberSubsystem;

        addRequirements(pivotSubsystem);

        SequentialCommandGroup dropCommand = new WaitCommand(3).andThen(new InstantCommand(() -> grabberSubsystem.toggle()));

        addCommands(
            new InstantCommand(() -> grabberSubsystem.grab()),
            new WaitCommand(0.2),
            new WaitCommand(5)
                .deadlineWith(
                    new WaitUntilCommand(() -> Math.abs(telescopeSubsystem.getTelescopePosition() - ArmConstants.telescopeMiddleSetpoint) < 0.05)
                        .andThen(new InstantCommand(() -> grabberSubsystem.toggle()))
                            .deadlineWith(
                                new RunCommand(() -> pivotSubsystem.setPivotPosition(0.015)),
                                new WaitUntilCommand(() -> pivotSubsystem.getPivotInRange(0.015, 0.012))
                                    .andThen(new RunCommand(() -> telescopeSubsystem.setTelescopePosition(ArmConstants.telescopeMiddleSetpoint))))
                        .andThen(new WaitCommand(0.5)
                            .andThen(new ParallelCommandGroup(
                                new RunCommand(() -> telescopeSubsystem.setTelescopePosition(0.03)),
                                new WaitUntilCommand(() -> telescopeSubsystem.getTelescopePosition() < 0.2)
                                    .andThen(new RunCommand(() -> pivotSubsystem.setPivotPosition(ArmConstants.elevatorUpPivotDownPosition)))))))

        );

    }
    
}
