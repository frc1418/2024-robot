// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class PivotSubsystem extends SubsystemBase {

    private CANSparkMax pivotMotor;
    private SparkAbsoluteEncoder pivotEncoder;

    private final NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
    private final NetworkTable table = ntInstance.getTable("/components/arm");

    private final NetworkTableEntry ntPivotPosition = table.getEntry("pivotPosition");
    private final NetworkTableEntry ntTargetPivotPosition = table.getEntry("targetPivotPosition");

    private PIDController pivotPidController = new PIDController(0, 0, 0);
    private ArmFeedforward armFeedforward = new ArmFeedforward(0, 0, 0);

    private double targetPos;
    private double lockPos;

    public PivotSubsystem(CANSparkMax pivotMotor) {
        this.pivotMotor = pivotMotor;
        this.pivotEncoder = pivotMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        this.pivotEncoder.setZeroOffset(ShooterConstants.PIVOT_OFFSET);

        pivotPidController.enableContinuousInput(0, 1);

        targetPos = pivotEncoder.getPosition();
        lockPos = pivotEncoder.getPosition();
    }

    public void setPivotMotor(double speed) {
        pivotMotor.set(speed);
    }

    public void setPivotPosition(double pos) {
        pivotMotor.setVoltage(armFeedforward.calculate(pos, 0) + pivotPidController.calculate(ntPivotPosition.getDouble(0), pos));
    }

    public void updatePivotPosition() {
        ntPivotPosition.setDouble(pivotEncoder.getPosition());
    }

    public double getPivotPosition() {
        return ntPivotPosition.getDouble(0);
    }

    @Override
    public void periodic() {
        ntPivotPosition.setDouble(pivotEncoder.getPosition());
        ntTargetPivotPosition.setDouble(targetPos);
    }

    public void setTargetPos(double targetPos) {
        this.targetPos = targetPos;
    }

    public double getTargetPos(){
        return targetPos;
    }

    public void setLockPos(double lockPos) {
        this.lockPos = lockPos;
    }

    public double getLockPos() {
        return lockPos;
    }
}
