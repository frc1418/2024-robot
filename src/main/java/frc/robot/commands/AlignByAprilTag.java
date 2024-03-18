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

    double targetRot;
    double approachAngle;

    boolean startedFieldCentric;

    PIDController speedController;

    SlewRateLimiter limitX = new SlewRateLimiter(1.5);
    SlewRateLimiter limitY = new SlewRateLimiter(1.5);

    public AlignByAprilTag(SwerveDriveSubsystem swerveDrive, LimelightSubsystem limelight, Odometry odometry, double targetX, double targetY, double P, double I, double D, double targetRot, double approachAngle) {

        this.swerveDrive = swerveDrive;
        this.limelight = limelight;
        this.odometry = odometry;
        this.targetX = targetX;
        this.targetY = targetY;
        this.targetRot = targetRot;
        this.approachAngle = approachAngle;
       
        speedController = new PIDController(P, I, D);
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

        System.out.println("Robot X: " + robotPose.getX());
        System.out.println("Robot Y: " + robotPose.getY());

        Pose2d targetPose;
        targetPose = new Pose2d(new Translation2d(targetX, -targetY), Rotation2d.fromDegrees(targetRot));
        
        double x;
        double y;
        double rot;

        double dx = robotPose.getX() - targetPose.getX();
        double dy =  robotPose.getY() - targetPose.getY();

        double distance = Math.hypot(dx, dy);
        double angleToTarget = Math.atan2(dx, dy) * 180 / Math.PI;

        rot = speedRotController.calculate(odometry.getPose().getRotation().getDegrees(), targetRot);

        double speed = -speedController.calculate(distance, 0);

        Rotation2d direction = Rotation2d.fromDegrees(180 + angleToTarget - odometry.getHeading() + targetRot - approachAngle);
        System.out.println(direction.getDegrees());

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