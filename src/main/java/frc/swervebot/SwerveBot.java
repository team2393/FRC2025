// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.swervebot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.led.ColorPair;
import frc.led.Comet;
import frc.led.Rainbow;
import frc.robot.OperatorInterface;
import frc.led.LEDRing;
// import frc.swervelib.AbsoluteSwerveCommand;
import frc.swervelib.RelativeSwerveCommand;
import frc.swervelib.ResetHeadingCommand;
import frc.swervelib.SwerveDrivetrain;
import frc.swervelib.SwerveOI;
// import frc.tools.AutoTools;
import frc.tools.CommandRobotBase;

/** Swerve demo robot */
public class SwerveBot extends CommandRobotBase
{
  private final SwervebotDrivetrain drivetrain = new SwervebotDrivetrain();
  private final Command relswerve = new RelativeSwerveCommand(drivetrain);
  // private final Command absswerve = new AbsoluteSwerveCommand(drivetrain);
  // private final Command center_on_tag = new CenterOnAprilTag(drivetrain);
  
  private final LEDRing ring = new LEDRing();

  private final SendableChooser<Command> autos = new SendableChooser<>();

  @Override
  public void robotInit()
  {
    super.robotInit();

    SwerveDrivetrain.MAX_METERS_PER_SEC = 1.5;
    SwerveDrivetrain.MAX_ROTATION_DEG_PER_SEC = 45;
    SwerveOI.forward_slew = new SlewRateLimiter(1.5);
    SwerveOI.side_slew = new SlewRateLimiter(1.5);
    SwerveOI.rotation_slew = new SlewRateLimiter(90);

    SwerveOI.reset();

    // ring.setDefaultCommand(
    //   new RepeatCommand(
    //     new SequentialCommandGroup(
    //       new ParallelRaceGroup(new WaitCommand(10), new Comet(ring)),
    //       new ParallelRaceGroup(new WaitCommand(10), new ColorPair(ring, Color.kDarkGreen, Color.kDarkGoldenrod))
    //     )));

    ring.setDefaultCommand(
      new Comet(ring).withTimeout(10)
                     .andThen(
                       new ColorPair(ring, Color.kDarkGreen, Color.kDarkGoldenrod).withTimeout(10)
                                                                                  .andThen(
                                                                                    new Rainbow(ring).withTimeout(10)
                                                                                          )
                             ).repeatedly()
                          );

    autos.setDefaultOption("Nothing", new PrintCommand("Do nothing"));

    // for (Command auto : AutoNoMouse.createAutoCommands(drivetrain))
    //   autos.addOption(auto.getName(), auto);
    // SmartDashboard.putData(autos);
  }
  
  @Override
  public void disabledPeriodic()
  {
    // AutoTools.indicateStart(drivetrain, autos.getSelected());
  }  

  @Override
  public void teleopInit()
  {
    // Bind buttons to commands
    drivetrain.setDefaultCommand(relswerve);
    // OperatorInterface.selectRelative().onTrue(relswerve);
    // OperatorInterface.selectAbsolute().onTrue(absswerve);
    OperatorInterface.resetHeading().onTrue(new ResetHeadingCommand(drivetrain));
    // SwerveOI.joystick.a().whileTrue(center_on_tag);

    // gadgetcommand.schedule();
  }

  @Override
  public void autonomousInit()
  {
    autos.getSelected().schedule();
  }
}
