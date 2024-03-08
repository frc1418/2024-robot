package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.common.Odometry;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class ShootByAprilTag extends Command {

    PIDController speedXController;
    PIDController speedYController;

    PIDController speedRotController;

    SwerveDriveSubsystem swerveDrive;
    LimelightSubsystem limelight;
    Odometry odometry;
    
    boolean startedFieldCentric;

    PIDController speedController;

    SlewRateLimiter limitX = new SlewRateLimiter(2);
    SlewRateLimiter limitY = new SlewRateLimiter(2);

    public ShootByAprilTag(SwerveDriveSubsystem swerveDrive, LimelightSubsystem limelight, Odometry odometry) {

        this.swerveDrive = swerveDrive;
        this.limelight = limelight;
        this.odometry = odometry;
       
        speedRotController = new PIDController(0.05, 0.002, 0);
        speedRotController.enableContinuousInput(-180, 180);

        addRequirements(swerveDrive, limelight);

    }


    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        this.startedFieldCentric = swerveDrive.getFieldCentric();
        this.swerveDrive.setFieldCentric(false);

        swerveDrive.drive(limitX.calculate(0),limitY.calculate(0),0);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {


        Pose2d robotPose = new Pose2d(
            new Translation2d(odometry.getPose().getX(), -odometry.getPose().getY()),
            odometry.getPose().getRotation());

        Pose2d targetPose;
        Rotation2d targetRot = limelight.getRotationToTargetPlane(); 
        targetPose = new Pose2d(new Translation2d(0, 0), targetRot);


        System.out.println("Robot X: " + robotPose.getX());
        System.out.println("Robot X again: " + targetPose.getX());
        
        double rot;

        double dx = robotPose.getX();
        double dy =  robotPose.getY();

        double distance = Math.hypot(dx, dy);
        double angleToTarget = Math.atan2(dx, dy) * 180 / Math.PI;

        rot = speedRotController.calculate(odometry.getPose().getRotation().getDegrees(), targetRot.getDegrees());
        System.out.println("Current Rot: " + odometry.getPose().getRotation().getDegrees());
        System.out.println("Target Rot: "+ targetRot.getDegrees());
        System.out.println("Angle to target: " + angleToTarget);

        // swerveDrive.drive(0, 0, rot);
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