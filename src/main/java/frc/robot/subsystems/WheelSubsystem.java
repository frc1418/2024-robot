package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WheelSubsystem extends SubsystemBase{

    private CANSparkMax angleMotor;
    private CANSparkMax speedMotor;
    private SparkAbsoluteEncoder turningEncoder;
    
    Translation2d location;

    
    public WheelSubsystem(CANSparkMax angleMotor, CANSparkMax speedMotor, Translation2d location) {

        this.angleMotor = angleMotor;
        this.speedMotor = speedMotor;
        this.turningEncoder =  angleMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        this.location = location;


    }

    public double getEncoderPosition(){
        return turningEncoder.getPosition();
    }


	public SwerveModulePosition getSwerveModulePosition() {
        return new SwerveModulePosition(
            speedMotor.getEncoder().getPosition(), Rotation2d.fromRotations(getEncoderPosition()));
	}
    
}
