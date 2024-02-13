// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {

    CANSparkMax leftMotor;
    CANSparkMax rightMotor;

    public ClimberSubsystem(CANSparkMax leftMotor, CANSparkMax rightMotor) {
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
    }

    public void climb(double speed) {
        leftMotor.set(speed);
        rightMotor.set(speed);
    }
}
