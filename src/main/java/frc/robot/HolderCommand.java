package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class HolderCommand extends Command
{
    private final Holder holder;
    private final boolean goingIn;
    private final NetworkTableEntry holder_voltage;
    private final Timer timer;

    public HolderCommand(Holder holder, boolean goingIn)
    {
        this.holder = holder;
        this.goingIn = goingIn;
        holder_voltage = SmartDashboard.getEntry("Holder Voltage");
        this.timer = new Timer();
        addRequirements(holder);
    }

    @Override
    public void initialize()
    {
        timer.restart();
    }

    @Override
    public void execute()
    {
        if (goingIn)
        {

            holder.setVoltage(-holder_voltage.getDouble(3));
        }
        else
        {
            holder.setVoltage(holder_voltage.getDouble(3));
        }
    }
    
    @Override
    public void end(boolean interrupted)
    {
        holder.setVoltage(0);
    }
}
