// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

  private CANSparkMax intakeMotor;

  public IntakeSubsystem(CANSparkMax intakeMotor) {
    this.intakeMotor = intakeMotor;
  }

  public void intake(double speed) {
    intakeMotor.set(speed);
  }
  
  public double getSpeed() {
    return intakeMotor.get();
  }
}
