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
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.common.Odometry;

public class SwerveDriveSubsystem extends SubsystemBase {
    
    private MaxWheelSubsystem backRight;
    private MaxWheelSubsystem backLeft;
    private MaxWheelSubsystem frontLeft;
    private MaxWheelSubsystem frontRight;

    private final NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
    private final NetworkTable table = ntInstance.getTable("/components/drivetrain");

    private final NetworkTableEntry ntBackRightAngleEncoder = table.getEntry("backRightAngleEncoder");
    private final NetworkTableEntry ntBackLeftAngleEncoder = table.getEntry("backLeftAngleEncoder");
    private final NetworkTableEntry ntFrontRightAngleEncoder = table.getEntry("frontRightAngleEncoder");
    private final NetworkTableEntry ntFrontLeftAngleEncoder = table.getEntry("frontLeftAngleEncoder");

    private final NetworkTableEntry ntIsFieldCentric = table.getEntry("isFieldCentric");

    private final NetworkTable odometryTable = ntInstance.getTable("/common/Odometry");
    private final NetworkTableEntry ntOdometryPose = odometryTable.getEntry("odometryPose");
    private final NetworkTableEntry ntVelocityBackRight = table.getEntry("wheelvelocitybackright");
    private final NetworkTableEntry ntVelocityBackLeft = table.getEntry("wheelvelocitybackleft");
    private final NetworkTableEntry ntVelocityFrontRight = table.getEntry("wheelvelocityfrontright");
    private final NetworkTableEntry ntVelocityFrontLeft = table.getEntry("wheelvelocityfrontleft");

    // private PIDController rotationController = new PIDController(0, 0, 0); 
    private PIDController rotationController = new PIDController(0.04, 0, 0); 

    
    private SwerveDriveKinematics kinematics;
    private Odometry odometry;

    public boolean fieldCentric = true;

    private double lockedRot = 0;

    public SwerveDriveSubsystem(MaxWheelSubsystem backRight, MaxWheelSubsystem backLeft, MaxWheelSubsystem frontRight, MaxWheelSubsystem frontLeft,
            SwerveDriveKinematics kinematics, Odometry odometry) {

        this.backRight = backRight;
        this.backLeft = backLeft;
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;

        this.kinematics = kinematics;
        this.odometry = odometry;

        this.ntIsFieldCentric.setBoolean(fieldCentric);
    }

    public void drive(double x, double y, double rot) {

        if(rot == 0){
            rot = rotationController.calculate(odometry.getHeading(), lockedRot);
        }
        else{
            lockedRot = odometry.getHeading();
        }

        //Caps pos or neg rotation speed
        if (Math.abs(rot) > DrivetrainConstants.ROTATION_SPEED_CAP) {
            rot = DrivetrainConstants.ROTATION_SPEED_CAP * Math.signum(rot);
        }

        ChassisSpeeds speeds = new ChassisSpeeds(x, y, rot);

        if (fieldCentric) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, odometry.getRotation2d());
        }
        drive(speeds);
    }

    public void drive(ChassisSpeeds speeds){
        // Convert to module states
        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);

        drive(moduleStates);
    }

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

    public void turtle() {
        frontRight.setAngle(Rotation2d.fromDegrees(-45));
        backLeft.setAngle(Rotation2d.fromDegrees(-45));
        
        backRight.setAngle(Rotation2d.fromDegrees(45));
        frontLeft.setAngle(Rotation2d.fromDegrees(45));
    }

    @Override
    public void periodic() {
        odometry.update(getPositions());

        ntOdometryPose.setString(odometry.getPose().toString());

        ntBackLeftAngleEncoder.setDouble(backLeft.getEncoderPosition());
        ntBackRightAngleEncoder.setDouble(backRight.getEncoderPosition());
        ntFrontLeftAngleEncoder.setDouble(frontLeft.getEncoderPosition());
        ntFrontRightAngleEncoder.setDouble(frontRight.getEncoderPosition());




        ntVelocityBackRight.setDouble(backRight.getSpeed());
        ntVelocityBackLeft.setDouble(backLeft.getSpeed());
        ntVelocityFrontRight.setDouble(frontRight.getSpeed());
        ntVelocityFrontLeft.setDouble(frontLeft.getSpeed());

    }

    public Command toggleFieldCentric() {
        return runOnce(() -> {
            fieldCentric = !fieldCentric;
            ntIsFieldCentric.setBoolean(fieldCentric);
        });
    }

    public void resetFieldCentric() {
        odometry.zeroHeading();
        resetLockRot();
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
}
