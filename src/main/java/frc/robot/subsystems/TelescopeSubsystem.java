package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class TelescopeSubsystem extends SubsystemBase {

    private TalonFX telescopeMotor;

    private final NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
    private final NetworkTable table = ntInstance.getTable("/components/arm");

    private final NetworkTableEntry ntTelescopeLength = table.getEntry("telescopeLength");
    
    public TelescopeSubsystem(TalonFX telescopeMotor, PivotSubsystem pivotSubsystem) {
        this.telescopeMotor = telescopeMotor;

        this.telescopeMotor.selectProfileSlot(0, 0);
        this.telescopeMotor.config_kP(0, 1.2);
        this.telescopeMotor.config_kI(0, 0);
        this.telescopeMotor.config_kD(0, 0.5);
        this.telescopeMotor.config_kF(0, 0);
        
        this.telescopeMotor.getSensorCollection().setIntegratedSensorPosition(0, 0);

    }


    public void setTelescopeMotor(double speed){
        telescopeMotor.set(ControlMode.PercentOutput, speed);
    }

    public void setTelescopePosition(double pos) {
        telescopeMotor.set(ControlMode.Position, -pos /ArmConstants.telescopeRotationToMeters);
    }

    @Override
    public void periodic() {
        updateTelescopeLength();
    }


    public void updateTelescopeLength() {
        ntTelescopeLength.setDouble(ArmConstants.telescopeRotationToMeters * telescopeMotor.getSensorCollection().getIntegratedSensorPosition());
    }

    public double getTelescopePosition() {
        return ntTelescopeLength.getDouble(0);
    }

    public void resetTelescopeEncoder() {
        telescopeMotor.getSensorCollection().setIntegratedSensorPosition(0, 0);
        ntTelescopeLength.setDouble(0);
    }
}
