package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class EjectCommand extends Command
{
    private Intake intake;
    private Timer timer;

    public EjectCommand(Intake intake)
    {
        this.intake = intake;
        this.timer = new Timer();
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
        System.out.println("ejecting...");
        intake.setVoltage(-3);
    }

    @Override
    public boolean isFinished()
    {
        return timer.hasElapsed(2);
    }

    @Override
    public void end(boolean interrupted)
    {
        System.out.println("finished ejecting");
        intake.setVoltage(0);
    }
}
