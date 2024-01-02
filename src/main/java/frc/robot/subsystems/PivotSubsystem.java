package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class PivotSubsystem extends SubsystemBase {

    private CANSparkMax pivotMotor;
    private SparkMaxAbsoluteEncoder pivotEncoder;

    private final NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
    private final NetworkTable table = ntInstance.getTable("/components/arm");

    private final NetworkTableEntry ntPivotPosition = table.getEntry("pivotPosition");
    private final NetworkTableEntry ntTelescopeLength = table.getEntry("telescopeLength");

    private PIDController pivotPidController = new PIDController(19, 3, 0);
    private ArmFeedforward armFeedforward = new ArmFeedforward(0, ArmConstants.startingPivotG, 0);

    private double targetPos;

    public PivotSubsystem(CANSparkMax pivotMotor) {
        this.pivotMotor = pivotMotor;
        
        this.pivotEncoder = pivotMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
        this.pivotEncoder.setZeroOffset(ArmConstants.pivotOffset);

        pivotPidController.enableContinuousInput(0, 1);

        targetPos = pivotEncoder.getPosition();
    }

    public void setPivotMotorVoltage(double speed) {
        
        pivotMotor.setVoltage(speed);
    }

    public void setPivotMotor(double speed) {
        pivotMotor.set(speed);
    }

    public void setPivotPosition(double pos) {
        pivotMotor.setVoltage(armFeedforward.calculate(pos, 0) + pivotPidController.calculate(ntPivotPosition.getDouble(0), pos));
    }

    @Override
    public void periodic() {
        updatePivotPosition();
        this.updateArmFeedForward(ArmConstants.startingPivotG + ArmConstants.pivotGPerTelescopeMeter*ntTelescopeLength.getDouble(0));
    }

    public void updatePivotPosition() {
        ntPivotPosition.setDouble(pivotEncoder.getPosition());
    }

    public double getPivotPosition() {
        return ntPivotPosition.getDouble(0);
    }

    public void updateArmFeedForward(double newG) {
        armFeedforward = new ArmFeedforward(0, newG, 0);
    }

    public boolean getPivotInRange(double setpoint, double buffer){
        return Math.abs(getPivotPosition() - setpoint) < buffer || Math.abs(getPivotPosition() - setpoint) > 1 - buffer;
    }

    public void setTargetPivot(double targetPos) {
        this.targetPos = targetPos;
    }

    public double getTargetPivot(){
        return targetPos;
    }
}
