package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeCommand extends Command
{
    private final Intake intake;
    private final NetworkTableEntry nt_speed;

    public IntakeCommand(Intake intake)
    {
        this.intake = intake;
        addRequirements(intake);
        nt_speed = SmartDashboard.getEntry("Intake Voltage");
        nt_speed.setDefaultDouble(5);
    }

    @Override
    public void execute()
    {
        System.out.println("intaking...");
        intake.setVoltage(nt_speed.getDouble(5));
    }

    @Override
    public boolean isFinished()
    {
        return intake.hasGamePiece()  &&  RobotBase.isSimulation();
    }

    @Override
    public void end(boolean interrupted)
    {
        System.out.println("finished intaking.");
        intake.setVoltage(0);
    }
}
