package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakeCommand extends Command
{
    private Intake intake;

    public IntakeCommand(Intake intake)
    {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void execute()
    {
        System.out.println("intaking...");
        intake.setVoltage(5);
    }

    @Override
    public boolean isFinished()
    {
        return intake.hasGamePiece();
    }

    @Override
    public void end(boolean interrupted)
    {
        System.out.println("finished intaking.");
        intake.setVoltage(0);
    }
}
