// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//Best angle for amp rn: 0.344
//Best speed for amp rn: 0.40

//Best angle for intake: 0.811

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
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
    private final NetworkTableEntry ntLockPivotPosition = table.getEntry("lockPivotPosition");
    private final NetworkTableEntry ntFF = table.getEntry("FF Volts");
    private final NetworkTableEntry ntP = table.getEntry("P Volts");
    private final NetworkTableEntry ntI = table.getEntry("I Volts");
    private final NetworkTableEntry ntD = table.getEntry("D Volts");

    private double PVal = 22;
    private double IVal = 27.5; 
    private double DVal = 3; 

    private PIDController pivotPidController = new PIDController(PVal, IVal, DVal);
    private PIDController P = new PIDController(PVal, 0, 0);
    private PIDController I = new PIDController(0, IVal, 0);
    private PIDController D = new PIDController(0, 0, DVal);

    private double kG = 0.142;

    private ArmFeedforward armFeedforward = new ArmFeedforward(0, kG, 0);

    private double targetPos;
    private double lockPos;

    private SlewRateLimiter limitP = new SlewRateLimiter(0.5);

    public PivotSubsystem(CANSparkMax pivotMotor) {
        this.pivotMotor = pivotMotor;
        this.pivotEncoder = pivotMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        this.pivotEncoder.setZeroOffset(ShooterConstants.PIVOT_OFFSET);

        pivotPidController.enableContinuousInput(0, 1);

        targetPos = MathUtil.clamp(pivotEncoder.getPosition(), 0.75, 0.992);
        lockPos = MathUtil.clamp(pivotEncoder.getPosition(), 0.75, 0.992);
    }

    public void setPivotMotor(double speed) {
        pivotMotor.set(speed);
    }

    public void setPivotPosition(double pos) {
        pivotMotor.setVoltage(limitP.calculate(armFeedforward.calculate(pos*2*Math.PI, 0) - pivotPidController.calculate(pivotEncoder.getPosition(), MathUtil.clamp(pos, 0.75, 0.992))));
        System.out.println(pos);
        ntFF.setDouble(armFeedforward.calculate(pos*2*Math.PI, 0));
        ntP.setDouble(-P.calculate(pivotEncoder.getPosition(), pos));
        ntI.setDouble(-I.calculate(pivotEncoder.getPosition(), pos));
        ntD.setDouble(-D.calculate(pivotEncoder.getPosition(), pos));
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
        ntLockPivotPosition.setDouble(lockPos);
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

    public void changeTargetPos(double val) {
        targetPos += val;
    }
}
