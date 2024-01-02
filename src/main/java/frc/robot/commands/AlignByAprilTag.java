package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.LimelightDirections;
import frc.robot.common.Odometry;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class AlignByAprilTag extends CommandBase {

    PIDController speedXController;
    PIDController speedYController;

    PIDController speedRotController;

    SwerveDriveSubsystem swerveDrive;
    LimelightSubsystem limelight;
    Odometry odometry;

    double targetX;
    double targetY;

    boolean startedFieldCentric;

    ProfiledPIDController speedController;

    public AlignByAprilTag(SwerveDriveSubsystem swerveDrive, LimelightSubsystem limelight, Odometry odometry, double targetX, double targetY) {

        this.swerveDrive = swerveDrive;
        this.limelight = limelight;
        this.odometry = odometry;
        this.targetX = targetX;
        this.targetY = targetY;
       
        speedController = new ProfiledPIDController(1, 0, 0, new Constraints(0,0));
        speedRotController = new PIDController(0.05, 0, 0);
        speedRotController.enableContinuousInput(-180, 180);

        addRequirements(swerveDrive, limelight);

    }


    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        this.startedFieldCentric = swerveDrive.getFieldCentric();
        this.swerveDrive.setFieldCentric(false);

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        Pose2d robotPose = new Pose2d(
            new Translation2d(odometry.getPose().getY(), odometry.getPose().getX()),
            odometry.getPose().getRotation());

        Pose2d targetPose;
        if(limelight.getTargetRotation() == LimelightDirections.GRID_SIDE)
            targetPose = new Pose2d(new Translation2d(targetX, -targetY), Rotation2d.fromDegrees(limelight.getTargetRotation().angle()));
        else
            targetPose = new Pose2d(new Translation2d(targetX*1.45, targetY*1.3), Rotation2d.fromDegrees(limelight.getTargetRotation().angle()));

        double x;
        double y;
        double rot;

        double dx = robotPose.getX() - targetPose.getX();
        double dy =  robotPose.getY() - targetPose.getY();

        double distance = Math.hypot(dx, dy);
        double angleToTarget = Math.atan2(dx, dy) * 180 / Math.PI;

        rot = speedRotController.calculate(odometry.getPose().getRotation().getDegrees(), limelight.getTargetRotation().angle());
        
        double speed = -speedController.calculate(distance, 0);

        if(limelight.getTargetRotation() == LimelightDirections.SUBSTATION_SIDE)
            speed *= 1;

        Rotation2d direction = Rotation2d.fromDegrees(180 + angleToTarget - odometry.getHeading() + limelight.getTargetRotation().angle());

        x = direction.getCos() * speed;
        y = direction.getSin() * speed;

        if(limelight.getTargetRotation() == LimelightDirections.GRID_SIDE){
            y *= -1;
            x *= -1;
        }

        swerveDrive.drive(x, y, rot);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        System.out.println("END");
        this.swerveDrive.setFieldCentric(startedFieldCentric);
        swerveDrive.drive(0, 0, 0);
        
        
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    
    
}
