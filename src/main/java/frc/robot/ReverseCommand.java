package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** Like Eject, but different direction */
public class ReverseCommand extends Command
{
    private final Intake intake;

    public ReverseCommand(Intake intake)
    {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void execute()
    {
        intake.setVoltage(-SmartDashboard.getNumber("Eject Voltage", 3));
    }

    @Override
    public void end(boolean interrupted)
    {
        intake.setVoltage(0);
    }
}
