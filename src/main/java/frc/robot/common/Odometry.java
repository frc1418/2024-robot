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



public class Odometry {
    private final SwerveDriveOdometry odometry;
    private final AHRS gyro;
    private SwerveModulePosition[] modulePositions;

    private final NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
    private final NetworkTable table = ntInstance.getTable("/components/Odometry");
    private final NetworkTableEntry heading = table.getEntry("heading");


    private final NetworkTableEntry pitch = table.getEntry("pitch");
    private final NetworkTableEntry roll = table.getEntry("roll");

    private Pose2d pose;

    public Odometry(
            AHRS gyro,
            SwerveDriveOdometry odometry, SwerveModulePosition[] modulePositions) {
        this.gyro = gyro;
        this.odometry = odometry;
        this.modulePositions = modulePositions;
        this.pose = new Pose2d();

        pitch.setDefaultDouble(0);
        roll.setDefaultDouble(0);
        zeroHeading();
    }

    public void update(SwerveModulePosition[] newPositions) {
        // Get the rotation of the robot from the gyro.
        var gyroAngle = gyro.getRotation2d();

        pose = odometry.update(gyroAngle, newPositions);

        modulePositions = newPositions;

        heading.setDouble(gyro.getRotation2d().getDegrees());
        pitch.setDouble(getPitch().getDegrees());
        roll.setDouble(getRoll().getDegrees());
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

    public Rotation2d getPitch() {
        // switched because of roborio placement
        return Rotation2d.fromDegrees(gyro.getRoll());
    }

    public Rotation2d getRoll() {
        // switched because of roborio placement
        return Rotation2d.fromDegrees(gyro.getPitch());
    }

    public Rotation2d getInclineAngle() {
        double y = getPitch().getRadians();
        double x = getRoll().getRadians();

        // vector math: treat pitch and roll values as 3d vectors
        // find cross product for normal line of plane,
        // use cosine angle rule for normal line and normal line's "shadow"
        // (same x and y, but z is 0)
        double rad = (Math.PI / 2) - Math.acos(Math.sqrt(
            (Math.pow(Math.cos(y) * Math.sin(x), 2) + Math.pow(Math.sin(y) * Math.cos(x), 2)) /
            (Math.pow(Math.cos(y), 2) + Math.pow(Math.sin(y) * Math.cos(x), 2))));
        
        return Rotation2d.fromRadians(rad).unaryMinus();
    }

    public Rotation2d getInclineDirection() {
        double y = getPitch().getRadians();
        double x = getRoll().getRadians();
        if (y == 0 && x == 0)
            // if robot is perfectly level, set direction to 0
            return new Rotation2d();
        else {
            // otherwise, angle is arctan of pitch and roll values
            double rad = Math.atan(Math.tan(x) / Math.tan(y));
            if (y < 0)
                return Rotation2d.fromRadians(rad + Math.PI);
            return Rotation2d.fromRadians(rad);
        }
    }
}
