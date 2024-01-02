package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.common.Odometry;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class LevelChargingStationCommand extends SequentialCommandGroup {

    public LevelChargingStationCommand(Odometry odometry, SwerveDriveSubsystem swerveDriveSubsystem, LedSubsystem ledSubsystem) {

        System.out.println("LEVELING");

        PIDCommand balanceRobot = new PIDCommand(
            new PIDController(0.026, 0, 0.008),
            () -> odometry.getInclineAngle().getDegrees(),
            0,
            (x) -> {
                System.out.println("BALANCING: " + x);
                swerveDriveSubsystem.strafe(odometry.getInclineDirection(), -x);
                if (Math.abs(x) < 0.05)
                    ledSubsystem.setDockedColor();
                else
                    ledSubsystem.setBalancingColor();
            },
            swerveDriveSubsystem);

        addCommands(balanceRobot);

    }
    
}
