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

public class AlignByAprilTag extends Command {

    PIDController speedXController;
    PIDController speedYController;

    PIDController speedRotController;

    SwerveDriveSubsystem swerveDrive;
    LimelightSubsystem limelight;
    Odometry odometry;

    double targetX;
    double targetY;

    boolean startedFieldCentric;

    PIDController speedController;

    SlewRateLimiter limitX = new SlewRateLimiter(0.5);
    SlewRateLimiter limitY = new SlewRateLimiter(0.5);

    public AlignByAprilTag(SwerveDriveSubsystem swerveDrive, LimelightSubsystem limelight, Odometry odometry, double targetX, double targetY) {

        this.swerveDrive = swerveDrive;
        this.limelight = limelight;
        this.odometry = odometry;
        this.targetX = targetX;
        this.targetY = targetY;
       
        speedController = new PIDController(1, 0, 0);
        speedRotController = new PIDController(0.05, 0.001, 0);
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
            new Translation2d(odometry.getPose().getY(), odometry.getPose().getX()),
            odometry.getPose().getRotation());

        //TODO: Check x and y of robot pose to make sure odometry not switched here and in limelight

        System.out.println("Robot X: " + robotPose.getX());
        System.out.println("Robot Y: " + robotPose.getY());

        Pose2d targetPose;
        targetPose = new Pose2d(new Translation2d(targetX, -targetY), Rotation2d.fromDegrees(limelight.getTargetRotation().angle()));
        
        double x;
        double y;
        double rot;

        double dx = robotPose.getX() - targetPose.getX();
        double dy =  robotPose.getY() - targetPose.getY();

        double distance = Math.hypot(dx, dy);
        double angleToTarget = Math.atan2(dx, dy) * 180 / Math.PI;

        System.out.println("dx: " + dx);

        System.out.println("dy: " + dy);

        System.out.println("rotation: " + odometry.getPose().getRotation().getDegrees());

        rot = speedRotController.calculate(odometry.getPose().getRotation().getDegrees(), limelight.getTargetRotation().angle());

        double speed = -speedController.calculate(distance, 0);

        Rotation2d direction = Rotation2d.fromDegrees(180 + angleToTarget - odometry.getHeading() + limelight.getTargetRotation().angle());

        x = direction.getCos() * speed;
        y = direction.getSin() * speed;

        swerveDrive.drive(limitX.calculate(x), limitY.calculate(y), rot);        
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