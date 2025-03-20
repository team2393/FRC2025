package frc.robot;

import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase
{
    private final TalonFX motor;
    private final DigitalInput sensor;
    private final NetworkTableEntry nt_gamepiece;

    public Intake()
    {
        motor = new TalonFX(RobotMap.INTAKE);
        TalonFXConfiguration config = new TalonFXConfiguration()
            .withOpenLoopRamps(new OpenLoopRampsConfigs().withVoltageOpenLoopRampPeriod(0.3));
        motor.getConfigurator().apply(config);    
        motor.clearStickyFaults();
        motor.setNeutralMode(NeutralModeValue.Brake);

        sensor = new DigitalInput(RobotMap.INTAKE_SENSOR);
        nt_gamepiece = SmartDashboard.getEntry("Gamepiece");
    }

    @Override
    public void periodic()
    {
        nt_gamepiece.setBoolean(hasGamePiece());
    }

    public boolean hasGamePiece()
    {
        return !sensor.get();
    }

    public void setVoltage(double voltage)
    {
        // Positive voltage for intaking
        motor.setVoltage(-voltage);
    }
}
