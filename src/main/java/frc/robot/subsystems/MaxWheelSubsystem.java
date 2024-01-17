package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WheelConstants;

public class MaxWheelSubsystem extends SubsystemBase{

    private CANSparkMax angleMotor;
    private CANSparkMax speedMotor;
    private SparkAbsoluteEncoder turningEncoder;
  
    private double targetSpeed = 0;

    private SparkPIDController speedPIDController;
    private PIDController anglePidController;
    
    Translation2d location;

    double angleSetpoint = 0;

    
    public MaxWheelSubsystem(CANSparkMax angleMotor, CANSparkMax speedMotor, Translation2d location) {

        this.angleMotor = angleMotor;
        this.speedMotor = speedMotor;
        this.turningEncoder =  angleMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        this.location = location;

        this.speedPIDController = this.speedMotor.getPIDController();
        speedPIDController.setP(0);
        speedPIDController.setI(0);
        speedPIDController.setD(0);
        speedPIDController.setFF(0.25);

        this.speedMotor.getEncoder().setPosition(0);
        this.speedMotor.getEncoder().setPositionConversionFactor(WheelConstants.ROTATIONS_TO_METERS);
        this.speedMotor.getEncoder().setVelocityConversionFactor(this.speedMotor.getEncoder().getPositionConversionFactor()/60.0);


        this.anglePidController = new PIDController(1.5, 0, 0);
        anglePidController.enableContinuousInput(0, 1);
        anglePidController.setTolerance(1.0/360);
    }

    public double getEncoderPosition(){
        return turningEncoder.getPosition();
    }


	public SwerveModulePosition getSwerveModulePosition() {
        return new SwerveModulePosition(
            speedMotor.getEncoder().getPosition(), Rotation2d.fromRotations(getEncoderPosition()));
	}

    public void drive(SwerveModuleState state)
    {
        SwerveModuleState optimizedState = SwerveModuleState.optimize(state, Rotation2d.fromRotations(
            getEncoderPosition()
        ));

        targetSpeed = optimizedState.speedMetersPerSecond;

        speedPIDController.setReference(targetSpeed, ControlType.kVelocity);

        Rotation2d angle = optimizedState.angle;
        setAngle(angle);
    }

    public void setAngle(Rotation2d angle){

        double pidOutput = anglePidController.calculate(getEncoderPosition(), angle.getRotations());
        double clampPidOutput = MathUtil.clamp(pidOutput, -1, 1);

        if(!anglePidController.atSetpoint())
            angleSetpoint = clampPidOutput;
        else
            angleSetpoint = 0;

        
        angleMotor.set(angleSetpoint);
    }

    public double getSpeed()
    {
        return speedMotor.getEncoder().getVelocity();
    }

    public SparkAbsoluteEncoder getTurningEncoder(){
        return turningEncoder;
    }

    
}
