// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {

    CANSparkMax leftMotor;
    CANSparkMax rightMotor;
    RelativeEncoder leftMotorEncoder;
    RelativeEncoder rightMotorEncoder;

    private final NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
    private final NetworkTable table = ntInstance.getTable("/components/arm");

    private final NetworkTableEntry ntLeftCLimbingEncoder = table.getEntry("leftClimbingEncoder");
    private final NetworkTableEntry ntRightCLimbingEncoder = table.getEntry("rightClimbingEncoder");


    public ClimberSubsystem(CANSparkMax leftMotor, CANSparkMax rightMotor) {
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        leftMotorEncoder = this.leftMotor.getEncoder();
        rightMotorEncoder = this.rightMotor.getEncoder();
    }

    public void climb(double speed) {
        leftMotor.set(-speed);
        rightMotor.set(speed);
    }

    public void stopClimbing(){
        leftMotor.set(-0.01);
        rightMotor.set(0.01);
    }

    public void setPosition (double leftPos, double rightPos)
    {
        leftMotorEncoder.setPosition(leftPos);
        rightMotorEncoder.setPosition(rightPos);
    }

    @Override
    public void periodic() {
        ntLeftCLimbingEncoder.setDouble(leftMotorEncoder.getPosition());
        ntRightCLimbingEncoder.setDouble(rightMotorEncoder.getPosition());

    }
}
