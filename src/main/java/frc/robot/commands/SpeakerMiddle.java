package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.AlignByAprilTag;
import frc.robot.common.Odometry;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class SpeakerMiddle extends SequentialCommandGroup {

    public SpeakerMiddle(PivotSubsystem pivotSubsystem, LimelightSubsystem limelight, SwerveDriveSubsystem swerveDrive, Odometry odometry) {

        final AlignByAprilTag alignAtSpeakerCenter = new AlignByAprilTag(swerveDrive, limelight, odometry, 0.04, -1.55, 1, 0.04, 0.1, 0, 0);

        addCommands(
            alignAtSpeakerCenter
                .deadlineWith(
                    // new WaitUntilCommand(() -> limelight.getDistance() < 3)
                    //     .andThen(
                new InstantCommand(() -> pivotSubsystem.setLockPos(0.823))
                        // )
                )
        );

    }
    
}