package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {

    NetworkTableInstance nt = NetworkTableInstance.getDefault();
    NetworkTable ntArm = nt.getTable("/components/arm");
    NetworkTableEntry ntArmAngle = nt.getEntry("pivotPosition");
    NetworkTableEntry ntArmLength = nt.getEntry("telescopeLength");


    DigitalInput topLimitSwitch = new DigitalInput(3);
    DigitalInput bottomLimitSwitch = new DigitalInput(4);
    
    private CANSparkMax elevatorMotor;


    public ElevatorSubsystem(CANSparkMax elevatorMotor) {
        this.elevatorMotor = elevatorMotor;

        elevatorMotor.getPIDController().setP(0);
        elevatorMotor.getPIDController().setI(0);
        elevatorMotor.getPIDController().setD(0);
    }

    public void setElevatorMotor(double speed) {
        if (bottomLimitSwitch.get() && speed < 0)
        {
            System.out.println("NO MORE DOWN");
            elevatorMotor.set(0);
        }
        else if (topLimitSwitch.get() && speed > 0)
        {
            System.out.println("NO MORE UP");
            elevatorMotor.set(0);
        }
        else
        {
            System.out.println("MOVING");
            elevatorMotor.set(speed);
        }
    }
    
}
