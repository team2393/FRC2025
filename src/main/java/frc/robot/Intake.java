package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase
{
    private final TalonFX motor;
    private final DigitalInput sensor;

    public Intake()
    {
        motor = new TalonFX(RobotMap.INTAKE);
        sensor = new DigitalInput(RobotMap.INTAKE_SENSOR);
    }

    public boolean hasGamePiece()
    {
        // TODO: Check polarity
        return sensor.get();
    }

    public void setVoltage(double voltage)
    {
        // Positive voltage for intaking
        motor.setVoltage(voltage);
    }
}
