package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {

    private CANSparkMax pivotMotor;
    private TalonFX telescopeMotor;
    private SparkMaxAbsoluteEncoder pivotEncoder;

    private final NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
    private final NetworkTable table = ntInstance.getTable("/components/arm");

    private final NetworkTableEntry ntPivotPosition = table.getEntry("pivotPosition");
    private final NetworkTableEntry ntTelescopeLength = table.getEntry("telescopeLength");

    private PIDController pivotPidController = new PIDController(19, 3, 0);//new PIDController(18, 0, 0);
    private ArmFeedforward armFeedforward = new ArmFeedforward(0, ArmConstants.startingPivotG, 0);

    public ArmSubsystem(CANSparkMax pivotMotor, TalonFX telescopeMotor) {
        this.pivotMotor = pivotMotor;
        this.telescopeMotor = telescopeMotor;

        this.telescopeMotor.selectProfileSlot(0, 0);
        this.telescopeMotor.config_kP(0, 1);
        this.telescopeMotor.config_kI(0, 0);
        this.telescopeMotor.config_kD(0, 0);
        this.telescopeMotor.config_kF(0, 0);
        
        this.pivotEncoder = pivotMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
        this.pivotEncoder.setZeroOffset(ArmConstants.pivotOffset);

        pivotPidController.enableContinuousInput(0, 1);

        this.telescopeMotor.getSensorCollection().setIntegratedSensorPosition(0, 0);

    }
    
}