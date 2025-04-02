package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class EjectCommand extends Command
{
    private final Intake intake;
    private final Timer timer;
    private final NetworkTableEntry nt_speed;

    public EjectCommand(Intake intake)
    {
        this.intake = intake;
        this.timer = new Timer();
        nt_speed = SmartDashboard.getEntry("Eject Voltage");
        nt_speed.setDefaultDouble(3);
        addRequirements(intake);
    }

    @Override
    public void initialize()
    {
        timer.restart();
    }

    @Override
    public void execute()
    {
        intake.setVoltage(nt_speed.getDouble(3));
    }

    @Override
    public boolean isFinished()
    {
        return timer.hasElapsed(0.5);
    }

    @Override
    public void end(boolean interrupted)
    {
        intake.setVoltage(0);
    }
}
