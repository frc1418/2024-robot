// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

  private CANSparkMax bottomLeftWheel;
  private CANSparkMax bottomRightWheel;
  private CANSparkMax topLeftWheel;
  private CANSparkMax topRightWheel;
  private CANSparkMax topWheel;

  public ShooterSubsystem(
    CANSparkMax bottomLeftWheel, 
    CANSparkMax bottomRightWheel, 
    CANSparkMax topLeftWheel, 
    CANSparkMax topRightWheel,
    CANSparkMax topWheel) 
  {
    this.bottomLeftWheel = bottomLeftWheel;
    this.bottomRightWheel = bottomRightWheel;
    this.topLeftWheel = topLeftWheel;
    this.topRightWheel = topRightWheel;
    this.topWheel = topWheel;
  }

  public void shoot(double speed) {
    //Forward is counter-clockwise
    bottomLeftWheel.set(speed);
    //Forward is clockwise
    bottomRightWheel.set(speed);
    //Forward is clockwise
    topLeftWheel.set(speed);
    //Forward is counter-clockwise
    topRightWheel.set(speed);
  }

  public void feed(double speed) {
    topWheel.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
