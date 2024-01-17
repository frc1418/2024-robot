package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;

import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WheelConstants;

public class MarkWheelSubsystem extends SubsystemBase{

    private CANSparkMax angleMotor;
    private CANSparkMax speedMotor;
    private AnalogEncoder turningEncoder;

    // private final NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
    // private final NetworkTable table = ntInstance.getTable("/components/drivetrain");

    // private final NetworkTableEntry ntSpeedTarget = table.getEntry("speedTarget");
    // private final NetworkTableEntry ntVelocity = table.getEntry("wheelvelocity");

  
    private double targetSpeed = 0;

    private SparkPIDController speedPIDController;
    private PIDController anglePidController;
    
    Translation2d location;

    double angleSetpoint = 0;

    
    public MarkWheelSubsystem(CANSparkMax angleMotor, CANSparkMax speedMotor, AnalogEncoder turningEncoder, Translation2d location) {

        this.angleMotor = angleMotor;
        this.speedMotor = speedMotor;
        this.turningEncoder =  turningEncoder;
        this.location = location;

        this.speedPIDController = this.speedMotor.getPIDController();
        speedPIDController.setP(0.005);
        speedPIDController.setI(0);
        speedPIDController.setD(0);
        speedPIDController.setFF(0.26);

        this.speedMotor.getEncoder().setPosition(0);
        this.speedMotor.getEncoder().setPositionConversionFactor(WheelConstants.ROTATIONS_TO_METERS);
        this.speedMotor.getEncoder().setVelocityConversionFactor(this.speedMotor.getEncoder().getPositionConversionFactor()/60.0);

        // this.anglePidController = new PIDController(0.5, 0, 0);
        this.anglePidController = new PIDController(0, 0, 0);
        anglePidController.enableContinuousInput(0, 1);
        anglePidController.setTolerance(1.0/360);

        // ntSpeedTarget.setDouble(0);
        // ntVelocity.setDouble(0);
    }

    public double getEncoderPosition(){
        return turningEncoder.getAbsolutePosition();
    }

	public SwerveModulePosition getSwerveModulePosition() {
        return new SwerveModulePosition(
            speedMotor.getEncoder().getPosition(), Rotation2d.fromRotations(getEncoderPosition()));
	}


    public void drive(SwerveModuleState state)
    {
        SwerveModuleState optimizedState = state; //SwerveModuleState.optimize(state, Rotation2d.fromRotations(
            // getEncoderPosition()));

        targetSpeed = optimizedState.speedMetersPerSecond;
        speedPIDController.setReference(targetSpeed, ControlType.kVelocity);

        // ntSpeedTarget.setDouble(targetSpeed);
        // ntVelocity.setDouble(speedMotor.getEncoder().getVelocity());

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
    
    public CANSparkMax getSpeedMotor()
    {
        return speedMotor;
    }
}
