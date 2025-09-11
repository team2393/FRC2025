package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Holder extends SubsystemBase
{
    private final TalonFX motor;

    public Holder()
    {
        motor = new TalonFX(RobotMap.HOLDER);
        TalonFXConfiguration config = new TalonFXConfiguration()
            .withOpenLoopRamps(new OpenLoopRampsConfigs().withVoltageOpenLoopRampPeriod(0.3));
        motor.getConfigurator().apply(config);    
        motor.clearStickyFaults();
        motor.setNeutralMode(NeutralModeValue.Brake);
        SmartDashboard.putNumber("Holder Voltage", 3);
    }

    public void setVoltage(double voltage)
    {
        // Positive voltage for intaking
        motor.setVoltage(voltage);
    }
}
