// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.swervelib.RelativeSwerveCommand;
import frc.swervelib.ResetHeadingCommand;
import frc.swervelib.SwerveDrivetrain;
import frc.swervelib.SwerveOI;
import frc.tools.AutoTools;
import frc.tools.CommandRobotBase;

/** FRC2025 robot */
public class Robot extends CommandRobotBase
{
  private final RobotDrivetrain drivetrain = new RobotDrivetrain();
  private final Command relswerve = new RelativeSwerveCommand(drivetrain);
    
  private final SendableChooser<Command> autos = new SendableChooser<>();

  public Robot()
  {
    // Configure speed: Slower, smoother movement for practice?
    SwerveOI.MAX_METERS_PER_SEC = SwerveDrivetrain.MAX_METERS_PER_SEC = 1.5;
    SwerveOI.MAX_ROTATION_DEG_PER_SEC = SwerveDrivetrain.MAX_ROTATION_DEG_PER_SEC = 45;
    SwerveOI.forward_slew = new SlewRateLimiter(1.5);
    SwerveOI.side_slew = new SlewRateLimiter(1.5);
    SwerveOI.rotation_slew = new SlewRateLimiter(90);

    SwerveOI.reset();
    autos.setDefaultOption("Nothing", new PrintCommand("Do nothing"));

    // for (Command auto : AutoNoMouse.createAutoCommands(drivetrain))
    //   autos.addOption(auto.getName(), auto);
    SmartDashboard.putData(autos);
  }
  
  @Override
  public void disabledPeriodic()
  {
    AutoTools.indicateStart(drivetrain, autos.getSelected());
  }  

  @Override
  public void teleopInit()
  {
    // Bind buttons to commands
    drivetrain.setDefaultCommand(relswerve);
    SwerveOI.selectRelative().onTrue(relswerve);
    SwerveOI.resetDrivetrain().onTrue(new ResetHeadingCommand(drivetrain));
  }

  @Override
  public void autonomousInit()
  {
    autos.getSelected().schedule();
  }
}
