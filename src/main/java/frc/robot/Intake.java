package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;

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
        sensor = new DigitalInput(RobotMap.INTAKE_SENSOR);
        nt_gamepiece = SmartDashboard.getEntry("Gamepiece");
    }

    public boolean hasGamePiece()
    {
        // TODO: Check polarity
        boolean have_gamepiece = sensor.get();
        nt_gamepiece.setBoolean(have_gamepiece);
        return have_gamepiece;
    }

    public void setVoltage(double voltage)
    {
        // Positive voltage for intaking
        motor.setVoltage(voltage);
    }
}
