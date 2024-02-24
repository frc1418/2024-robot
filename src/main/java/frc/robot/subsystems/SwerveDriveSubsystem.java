package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriverConstants;
import frc.robot.common.Odometry;

public class SwerveDriveSubsystem extends SubsystemBase {
    
    private MaxWheelModule backRight;
    private MaxWheelModule backLeft;
    private MaxWheelModule frontLeft;
    private MaxWheelModule frontRight;

    private final NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
    private final NetworkTable table = ntInstance.getTable("/components/drivetrain");

    private final NetworkTableEntry ntBackRightAngleEncoder = table.getEntry("backRightAngleEncoder");
    private final NetworkTableEntry ntBackLeftAngleEncoder = table.getEntry("backLeftAngleEncoder");
    private final NetworkTableEntry ntFrontRightAngleEncoder = table.getEntry("frontRightAngleEncoder");
    private final NetworkTableEntry ntFrontLeftAngleEncoder = table.getEntry("frontLeftAngleEncoder");

    private final NetworkTableEntry ntIsFieldCentric = table.getEntry("isFieldCentric");

    private final NetworkTable odometryTable = ntInstance.getTable("/common/Odometry");
    private final NetworkTableEntry ntOdometryPose = odometryTable.getEntry("odometryPose");

    private PIDController rotationController = new PIDController(0.04, 0, 0); 

    private SwerveDriveKinematics kinematics;
    private Odometry odometry;

    public boolean fieldCentric = true;

    private double lockedRot = 0;

    public SwerveDriveSubsystem(MaxWheelModule backRight, MaxWheelModule backLeft, MaxWheelModule frontRight, MaxWheelModule frontLeft,
            SwerveDriveKinematics kinematics, Odometry odometry) {

        this.backRight = backRight;
        this.backLeft = backLeft;
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;

        this.kinematics = kinematics;
        this.odometry = odometry;

        this.ntIsFieldCentric.setBoolean(fieldCentric);
    }

    //Initial drive method, maintains rotation and passes into ChassisSpeeds
    public void drive(double x, double y, double rot) {

        if(rot == 0){
            rot = rotationController.calculate(odometry.getHeading(), lockedRot);
        }
        else{
            lockedRot = odometry.getHeading();
        }
        if (Math.abs(rot) > DriverConstants.ROTATION_SPEED_CAP) {
            rot = DriverConstants.ROTATION_SPEED_CAP*Math.signum(rot);
        }

        ChassisSpeeds speeds = new ChassisSpeeds(x, y, rot);

        if (fieldCentric) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, odometry.getRotation2d());
        }
        drive(speeds);
    }
    
    // Second method passing to moduleStates
    public void drive(ChassisSpeeds speeds){
        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);

        drive(moduleStates);
    }

    //Third drive method, passing states to each module
    public void drive (SwerveModuleState[] moduleStates){
        SwerveModuleState frontLeftState = moduleStates[0];
        SwerveModuleState frontRightState = moduleStates[1];
        SwerveModuleState backLeftState = moduleStates[2];
        SwerveModuleState backRightState = moduleStates[3];

        frontLeft.drive(frontLeftState);
        frontRight.drive(frontRightState);
        backLeft.drive(backLeftState);
        backRight.drive(backRightState);
    }

    //Locks the robot's angles to prevent movement
    public void turtle() {
        frontRight.setAngle(Rotation2d.fromDegrees(-45));
        backLeft.setAngle(Rotation2d.fromDegrees(-45));
        
        backRight.setAngle(Rotation2d.fromDegrees(45));
        frontLeft.setAngle(Rotation2d.fromDegrees(45));
    }

    //Updates the network tables and odometry
    @Override
    public void periodic() {
        odometry.update(getPositions());

        ntOdometryPose.setString(odometry.getPose().toString());

        ntBackLeftAngleEncoder.setDouble(backLeft.getEncoderPosition());
        ntBackRightAngleEncoder.setDouble(backRight.getEncoderPosition());
        ntFrontLeftAngleEncoder.setDouble(frontLeft.getEncoderPosition());
        ntFrontRightAngleEncoder.setDouble(frontRight.getEncoderPosition());
    }

    public Command toggleFieldCentric() {
        return Commands.runOnce(() -> {
            fieldCentric = !fieldCentric;
            ntIsFieldCentric.setBoolean(fieldCentric);
        });
    }

    public Command resetFieldCentric() {
        return Commands.runOnce(() -> {
            odometry.zeroHeading();
            resetLockRot();
            odometry.setAngleOffset(180);
        });
    }

    public SwerveModulePosition[] getPositions() {
        return new SwerveModulePosition[] {
            frontLeft.getSwerveModulePosition(),
            frontRight.getSwerveModulePosition(),
            backLeft.getSwerveModulePosition(),
            backRight.getSwerveModulePosition()
        };
    }
    
    public void resetLockRot() {
        lockedRot = odometry.getHeading();
    }

    public boolean getFieldCentric() {
        return(fieldCentric);
    }

    public void setFieldCentric(boolean fieldCentric) {
        this.fieldCentric = fieldCentric;
    }
}
