// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.swervelib.RelativeSwerveCommand;
import frc.swervelib.ResetHeadingCommand;
import frc.swervelib.StopCommand;
import frc.swervelib.SwerveDrivetrain;
import frc.swervelib.SwerveOI;
import frc.swervelib.SwerveToPositionCommand;
import frc.tools.AutoTools;
import frc.tools.CommandRobotBase;

/** FRC2025 robot */
public class Robot extends CommandRobotBase
{
  private final RobotDrivetrain drivetrain = new RobotDrivetrain();
  private final Command relswerve = new RelativeSwerveCommand(drivetrain);
    
  private final SendableChooser<Command> autos = new SendableChooser<>();

  // TODO Use camera?
  // private final CameraHelper camera_helper = new CameraHelper();

  public Robot()
  {
    // Configure speed: Slower, smoother movement for practice
    // SwerveOI.MAX_METERS_PER_SEC = SwerveDrivetrain.MAX_METERS_PER_SEC = 1.5;
    // SwerveOI.MAX_ROTATION_DEG_PER_SEC = SwerveDrivetrain.MAX_ROTATION_DEG_PER_SEC = 45;
    // SwerveOI.forward_slew = new SlewRateLimiter(1.5);
    // SwerveOI.side_slew = new SlewRateLimiter(1.5);
    // SwerveOI.rotation_slew = new SlewRateLimiter(90);
    // AutoTools.config = new TrajectoryConfig(0.5, 1.0);
    // SwerveToPositionCommand.MAX_SPEED = 0.5;

    // Configure speed: Faster
    SwerveOI.MAX_METERS_PER_SEC = SwerveDrivetrain.MAX_METERS_PER_SEC = 1.0;
    SwerveOI.MAX_ROTATION_DEG_PER_SEC = SwerveDrivetrain.MAX_ROTATION_DEG_PER_SEC = 45;
    SwerveOI.forward_slew = new SlewRateLimiter(SwerveOI.MAX_METERS_PER_SEC);
    SwerveOI.side_slew = new SlewRateLimiter(SwerveOI.MAX_METERS_PER_SEC);
    SwerveOI.rotation_slew = new SlewRateLimiter(45);
    AutoTools.config = new TrajectoryConfig(SwerveOI.MAX_METERS_PER_SEC, SwerveOI.MAX_METERS_PER_SEC);
    SwerveToPositionCommand.MAX_SPEED = SwerveOI.MAX_METERS_PER_SEC;

    SwerveOI.reset();
    autos.setDefaultOption("Nothing", new PrintCommand("Do nothing"));
    for (Command auto : AutoNoMouse.createAutoCommands(drivetrain))
      autos.addOption(auto.getName(), auto);
    SmartDashboard.putData(autos);
    // Whenever something is selected, show its (optional) start position
    autos.onChange(selected -> AutoTools.indicateStart(drivetrain, selected));
  }
  
  @Override
  public void robotPeriodic()
  {
    super.robotPeriodic();
    // TODO Update camera?
    // camera_helper.updatePosition(drivetrain);
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
    drivetrain.setDefaultCommand(new StopCommand(drivetrain));
    autos.getSelected().schedule();
  }
}
