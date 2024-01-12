package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.common.Odometry;

public class SwerveDriveSubsystem extends SubsystemBase {
    
    private MarkWheelSubsystem backRight;
    private MarkWheelSubsystem backLeft;
    private MarkWheelSubsystem frontLeft;
    private MarkWheelSubsystem frontRight;

    private PIDController rotationController = new PIDController(0, 0, 0); // TODO: Tune PID

    
    private SwerveDriveKinematics kinematics;
    private Odometry odometry;

    public boolean fieldCentric = false;

    private double lockedRot = 0;

    public SwerveDriveSubsystem(MarkWheelSubsystem backRight, MarkWheelSubsystem backLeft, MarkWheelSubsystem frontRight, MarkWheelSubsystem frontLeft,
            SwerveDriveKinematics kinematics, Odometry odometry) {

        this.backRight = backRight;
        this.backLeft = backLeft;
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;

        this.kinematics = kinematics;
        this.odometry = odometry;

    }

    public void drive(double x, double y, double rot) {

        if(rot == 0){
            rot = rotationController.calculate(odometry.getHeading(), lockedRot);
        }
        else{
            lockedRot = odometry.getHeading();
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

    @Override
    public void periodic() {
        odometry.update(getPositions());
    }

    public SwerveModulePosition[] getPositions() {
        return new SwerveModulePosition[] {
            frontLeft.getSwerveModulePosition(),
            frontRight.getSwerveModulePosition(),
            backLeft.getSwerveModulePosition(),
            backRight.getSwerveModulePosition()
        };
    }
}
