package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GrabberSubsystem extends SubsystemBase {

    private DoubleSolenoid leftSolenoid;
    private DoubleSolenoid rightSolenoid;
    private Boolean grabberClosed = false;
    private LedSubsystem ledSubsystem;

    public GrabberSubsystem(DoubleSolenoid leftSolenoid, DoubleSolenoid rightSolenoid, LedSubsystem ledSubsystem) {
        this.leftSolenoid = leftSolenoid;
        this.rightSolenoid = rightSolenoid;
        leftSolenoid.set(Value.kForward);
        rightSolenoid.set(Value.kForward);

        this.ledSubsystem = ledSubsystem;
    }

    public void toggle(){
        leftSolenoid.toggle();
        rightSolenoid.toggle();
        System.out.println(leftSolenoid.get());
        grabberClosed = !grabberClosed;
        
        if (grabberClosed)
            ledSubsystem.setGrabberClosed();
        else
            ledSubsystem.setGrabberOpen();
    }

    public void grab(){
        leftSolenoid.set(Value.kReverse);
        rightSolenoid.set(Value.kReverse);

        grabberClosed = true;
        ledSubsystem.setGrabberClosed();
    }

    public void open() {
        leftSolenoid.set(Value.kForward);
        rightSolenoid.set(Value.kForward);

        grabberClosed = false;
        ledSubsystem.setGrabberOpen();
    }
    
}
