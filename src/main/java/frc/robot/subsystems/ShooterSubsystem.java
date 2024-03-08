// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

  private CANSparkMax leftWheel;
  private CANSparkMax rightWheel;

  public ShooterSubsystem(
    CANSparkMax leftWheel, 
    CANSparkMax rightWheel)  
  {
    this.leftWheel = leftWheel;
    this.rightWheel = rightWheel;
  }

  public void shoot(double speed) {
    //Forward is clockwise
    leftWheel.set(speed);
    //Forward is counter-clockwise
    rightWheel.set(speed);
  }
}
