// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you CAN modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

    private ShooterWheelSubsystem bottomLeftWheel;
    private ShooterWheelSubsystem bottomRightWheel;
    private ShooterWheelSubsystem topLeftWheel;
    private ShooterWheelSubsystem topRightWheel;
    private ShooterWheelSubsystem topWheel;


    public ShooterSubsystem(
        ShooterWheelSubsystem bottomLeftWheel, 
        ShooterWheelSubsystem bottomRightWheel, 
        ShooterWheelSubsystem topLeftWheel, 
        ShooterWheelSubsystem topRightWheel,
        ShooterWheelSubsystem topWheel)
        
    {
        this.bottomLeftWheel = bottomLeftWheel;
        this.bottomRightWheel = bottomRightWheel;
        this.topLeftWheel = topLeftWheel;
        this.topRightWheel = topRightWheel;
        this.topWheel = topWheel;
    }

  public void shoot(double speed1, double speed2) {
    //Forward is counter-clockwise
    bottomLeftWheel.spin(speed1);
    //Forward is clockwise
    bottomRightWheel.spin(-speed1);
    //Forward is clockwise
    topLeftWheel.spin(-speed1);
    //Forward is counter-clockwise
    topRightWheel.spin(speed1);
    
    topWheel.spin(speed2);
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
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
