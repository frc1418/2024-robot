package frc.robot.common;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.LimelightSubsystem;

public class Odometry extends SubsystemBase{
    private final SwerveDriveOdometry odometry;
    private final AHRS gyro;
    private SwerveModulePosition[] modulePositions;
    private LimelightSubsystem limelight;

    private final NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
    private final NetworkTable table = ntInstance.getTable("/components/Odometry");
    private final NetworkTableEntry heading = table.getEntry("heading");

    private Pose2d pose;

    public Odometry(
            AHRS gyro,
            SwerveDriveOdometry odometry, SwerveModulePosition[] modulePositions, LimelightSubsystem limelight) {
        this.gyro = gyro;
        this.odometry = odometry;
        this.modulePositions = modulePositions;
        this.limelight = limelight;
        this.pose = new Pose2d();
    }

    public void update(SwerveModulePosition[] newPositions) {
        // Get the rotation of the robot from the gyro.
        var gyroAngle = gyro.getRotation2d();

        pose = odometry.update(gyroAngle, newPositions);

        if (limelight.getIsDetecting() && DriverStation.isTeleop()){
                reset(
                    new Pose2d(new Translation2d(limelight.getXDistance(), limelight.getYDistance()), 
                    this.getPose().getRotation())
                );
        }

        modulePositions = newPositions;

        heading.setDouble(gyro.getRotation2d().getDegrees());
    }

    public void reset(Pose2d pose) {
        odometry.resetPosition(gyro.getRotation2d(), modulePositions, pose);
    }

    public void zeroHeading() {
        gyro.reset();
    }

    public void setAngleOffset(double offset) {
        gyro.setAngleAdjustment(offset);
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public double getHeading() {
        return gyro.getRotation2d().getDegrees();
    }

    public double getTurnRate() {
        return -gyro.getRate();
    }
    public Rotation2d getRotation2d() {
        return gyro.getRotation2d();
    }

    public AHRS getGyro() {
        return gyro;
    }
}
