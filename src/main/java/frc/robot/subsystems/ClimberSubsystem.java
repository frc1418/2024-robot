// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class ClimberSubsystem extends SubsystemBase {

    CANSparkMax leftMotor;
    CANSparkMax rightMotor;

    public ClimberSubsystem(CANSparkMax leftMotor, CANSparkMax rightMotor) {
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
    }

    public void stopClimbing() {
        if (RobotContainer.climbBrake) {
            // Once Friction on Left arm is fixed set back to -0.1
            leftMotor.set(-0.13);
            rightMotor.set(0.1);
        } else {
            leftMotor.set(0);
            rightMotor.set(0);
        }

    }

    public void climb(double speed) {
        double speed1 = speed + 0.1;
        leftMotor.set(-speed1);
        rightMotor.set(speed);
    }
}
