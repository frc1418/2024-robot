// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

    public LimelightSubsystem() {}


  @Override
  public void periodic() {
    
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
}
