package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

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

public class WheelSubsystem extends SubsystemBase{

    
    private CANSparkMax angleMotor;
    private CANSparkMax speedMotor;
    private PIDController anglePIDController;
    private SparkMaxPIDController speedPIDController;
    private SparkMaxAbsoluteEncoder turningEncoder;  
    
    Translation2d location;

    private double targetSpeed = 0;

    private double angleSetpoint = 0;
    private final NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
    private final NetworkTable table = ntInstance.getTable("/components/drivetrain");

    private final NetworkTableEntry ntSpeedTarget = table.getEntry("speedTarget");
    private final NetworkTableEntry ntVelocity = table.getEntry("wheelvelocity");

    public WheelSubsystem (CANSparkMax angleMotor, CANSparkMax speedMotor, AnalogEncoder turningEncoder, Translation2d location) {
        this.angleMotor = angleMotor;
        this.speedMotor = speedMotor;
        this.location = location;
        this.turningEncoder = angleMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);

        this.speedMotor.getEncoder().setPosition(0);
        this.speedMotor.getEncoder().setPositionConversionFactor(WheelConstants.ROTATIONS_TO_METERS);
        this.speedMotor.getEncoder().setVelocityConversionFactor(this.speedMotor.getEncoder().getPositionConversionFactor() / 60.0);
        this.speedPIDController = this.speedMotor.getPIDController();

        speedPIDController.setP(0.0001);
        speedPIDController.setI(0.00);
        speedPIDController.setD(0.00);
        speedPIDController.setFF(0.25);

        anglePIDController = new PIDController(1.5, 0, 0.04);
        anglePIDController.enableContinuousInput(0, 1);
        anglePIDController.setTolerance(1.0/360);


        ntSpeedTarget.setDouble(0);
        ntVelocity.setDouble(0);
    }

    public void drive (SwerveModuleState state) {
        
        SwerveModuleState optimizedState = SwerveModuleState.optimize(state,
            Rotation2d.fromRotations(getEncoderPosition()));
            
        targetSpeed = optimizedState.speedMetersPerSecond;
        speedPIDController.setReference(targetSpeed, ControlType.kVelocity);

        ntSpeedTarget.setDouble(targetSpeed);
        ntVelocity.setDouble(speedMotor.getEncoder().getVelocity());

        Rotation2d angle = optimizedState.angle;
        setAngle(angle);
        
    }

    public void setAngle(Rotation2d angle) {

        double pidOutput = anglePIDController.calculate(getEncoderPosition(), angle.getRotations());
        double clampedPidOutpt = MathUtil.clamp(pidOutput, -1, 1);
        
        if (!anglePIDController.atSetpoint())
            angleSetpoint = clampedPidOutpt;
        else
            angleSetpoint = 0;

        angleMotor.set(angleSetpoint);

    }

    public CANSparkMax getAngleMotor(){
        return angleMotor;
    }

    public CANSparkMax getSpeedMotor(){
        return speedMotor;
    }

    public SparkMaxAbsoluteEncoder getEncoder(){
        return turningEncoder;
    }
    public double getTargetSpeed() {
        return targetSpeed;
    }

    public double getEncoderPosition() {
        return turningEncoder.getPosition();
    }

    public double getangleSetpoint() {
        return angleSetpoint;
    }

    public Translation2d getLocation() {
        return location;
    }

    public double getDistanceDriven() {
        return speedMotor.getEncoder().getPosition();
    }

    public SwerveModulePosition getSwerveModulePosition() {
        return new SwerveModulePosition(
            speedMotor.getEncoder().getPosition(), Rotation2d.fromRotations(getEncoderPosition()));
    }

    public double getVelocity(){
        return speedMotor.getEncoder().getVelocity();
    }
}
