package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightDirections;

public class LimelightSubsystem extends SubsystemBase {

    private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    private NetworkTableEntry ntRobotposeTargetspace = table.getEntry("botpose_targetspace");
    private NetworkTableEntry ntCameraposeTargetspace = table.getEntry("camerapose_targetspace");

    // returns 0 if no target, 1, if target
    private NetworkTableEntry ntIsDetecting = table.getEntry("tv");

    private NetworkTableEntry ntID = table.getEntry("tid");

    private double distanceToTarget = 0;
    private double xToTarget;
    private double yToTarget;
    private double rotToTarget;
    private LimelightDirections targetRotation;
    
    
    public LimelightSubsystem() {

    }

    @Override
    public void periodic() {

        double[] botPosArray = ntRobotposeTargetspace.getDoubleArray(new double[6]);
        double [] camPosArray = ntCameraposeTargetspace.getDoubleArray(new double[6]);

        xToTarget = botPosArray[0];
        yToTarget = botPosArray[2];
        rotToTarget = camPosArray[4];

        if(DriverStation.getAlliance() == Alliance.Blue){
            if(ntID.getDouble(0) == 6 || ntID.getDouble(0) == 7 || ntID.getDouble(0) == 8){
                targetRotation = LimelightDirections.GRID_SIDE;
            } else if (ntID.getDouble(0) == 4){
                targetRotation = LimelightDirections.SUBSTATION_SIDE;
            }
        } else {
            if(ntID.getDouble(0) == 3 || ntID.getDouble(0) == 2 || ntID.getDouble(0) == 1){
                targetRotation = LimelightDirections.GRID_SIDE;
            } else if (ntID.getDouble(0) == 5){
                targetRotation = LimelightDirections.SUBSTATION_SIDE;
            }
        }

        distanceToTarget = Math.hypot(xToTarget, yToTarget);
    }

    public double getDistance() {
        return distanceToTarget;
    }

    public double getXDistance() {
        return xToTarget;
    }

    public double getYDistance() {
        return yToTarget;
    }


    public Rotation2d getRotationToTargetPlane() {
        return Rotation2d.fromDegrees(rotToTarget);
    }

    public boolean getIsDetecting() {
        int id = (int) ntID.getInteger(Integer.MAX_VALUE);
        boolean isDetecting = ntIsDetecting.getInteger(0) == 1 && id >= 1 && id <= 8;
        return isDetecting;
    }

    public LimelightDirections getTargetRotation() {
        return targetRotation;
    }

}
