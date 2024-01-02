package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;

public class DeliverUpperConeCommand extends SequentialCommandGroup {

    public DeliverUpperConeCommand(PivotSubsystem pivotSubsystem, TelescopeSubsystem telescopeSubsystem, GrabberSubsystem grabberSubsystem){

        addRequirements(pivotSubsystem);

        addCommands(
            new InstantCommand(() -> grabberSubsystem.grab()),
            new WaitCommand(0.2),
            new WaitCommand(5)
                .deadlineWith(
                    new WaitUntilCommand(() -> Math.abs(telescopeSubsystem.getTelescopePosition() - ArmConstants.telescopeOuterSetpoint) < AutoConstants.telescopeDeadband)
                        .andThen(new InstantCommand(() -> grabberSubsystem.toggle()))
                            .deadlineWith(
                                new RunCommand(() -> pivotSubsystem.setPivotPosition(AutoConstants.autoPivotUp)),
                                new WaitUntilCommand(() -> pivotSubsystem.getPivotInRange(AutoConstants.autoPivotUp, AutoConstants.pivotDeadband))
                                    .andThen(new RunCommand(() -> telescopeSubsystem.setTelescopePosition(ArmConstants.telescopeOuterSetpoint))))
                        .andThen(new WaitCommand(0.5)
                            .andThen(new ParallelCommandGroup(
                                new RunCommand(() -> telescopeSubsystem.setTelescopePosition(AutoConstants.autoTelescopeIn)),
                                new WaitUntilCommand(() -> telescopeSubsystem.getTelescopePosition() < AutoConstants.autoTelescopeWait)
                                    .andThen(new RunCommand(() -> pivotSubsystem.setPivotPosition(ArmConstants.elevatorUpPivotDownPosition)))))))
        );

    }
    
}
