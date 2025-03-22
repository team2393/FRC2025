// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.swervelib.AbsoluteSwerveCommand;
import frc.swervelib.RelativeSwerveCommand;
import frc.swervelib.ResetHeadingCommand;
import frc.swervelib.StopCommand;
import frc.swervelib.SwerveDrivetrain;
import frc.swervelib.SwerveOI;
import frc.swervelib.SwerveToPositionCommand;
import frc.tools.ApplyAdjustableSettingCommand;
import frc.tools.AutoTools;
import frc.tools.CommandRobotBase;

/** FRC2025 robot */
public class Robot extends CommandRobotBase
{
  private final RobotDrivetrain drivetrain = new RobotDrivetrain();
  private final Command relswerve = new RelativeSwerveCommand(drivetrain);
  private final Command absswerve = new AbsoluteSwerveCommand(drivetrain);
  private final SendableChooser<Command> autos = new SendableChooser<>();

  private final Lift lift = new Lift();
  private NetworkTableEntry nt_lift_setpoint;

  private final Intake intake = new Intake();

  // TODO pick correct field: k2025ReefscapeWelded, k2025ReefscapeAndyMark
  private final AprilTagFieldLayout tags = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

  // List all cameras in correct location
  private final CameraHelper cameras[] =
  {
    new CameraHelper(tags, "front", "Front Camera", 0.3, -0.03, 0.27, 0.0, -10.0),
  };

  public Robot()
  {
    // Configure speed
    SwerveOI.MAX_METERS_PER_SEC = SwerveDrivetrain.MAX_METERS_PER_SEC = 2.0; // 1 .. 3
    SwerveOI.MAX_ROTATION_DEG_PER_SEC = SwerveDrivetrain.MAX_ROTATION_DEG_PER_SEC = 180;
    SwerveOI.forward_slew = new SlewRateLimiter(3.0);
    SwerveOI.side_slew = new SlewRateLimiter(3.0);
    SwerveOI.rotation_slew = new SlewRateLimiter(360);
    AutoTools.config = new TrajectoryConfig(SwerveOI.MAX_METERS_PER_SEC, SwerveOI.MAX_METERS_PER_SEC);
    SwerveToPositionCommand.MAX_SPEED = SwerveOI.MAX_METERS_PER_SEC;

    PowerDistribution power = new PowerDistribution();
    power.clearStickyFaults();
    power.resetTotalEnergy();
    SmartDashboard.putData("PowerPanel", power);

    SwerveOI.reset();
    autos.setDefaultOption("Nothing", new PrintCommand("Do nothing"));
    for (Command auto : AutoNoMouse.createAutoCommands(drivetrain, tags, intake, lift))
      autos.addOption(auto.getName(), auto);
    SmartDashboard.putData(autos);
    // Whenever something is selected, show its (optional) start position
    autos.onChange(selected -> AutoTools.indicateStart(drivetrain, selected));

    // Commands that move lift to several adjustable positions
    Command park = new ApplyAdjustableSettingCommand("Lift Park", "Lift Park Setpoint", 0.00, "Lift Setpoint");
    SmartDashboard.putData(park);
    OperatorInterface.park().onTrue(park);

    Command lowest = new ApplyAdjustableSettingCommand("Lift Lowest",  "Lift Lowest Setpoint",  0.27, "Lift Setpoint");
    SmartDashboard.putData(lowest);
    OperatorInterface.lowestLift().onTrue(lowest);
    
    Command low = new ApplyAdjustableSettingCommand("Lift Low",  "Lift Low Setpoint",  0.52, "Lift Setpoint");
    SmartDashboard.putData(low);
    OperatorInterface.lowLift().onTrue(low);

    Command mid = new ApplyAdjustableSettingCommand("Lift Mid",  "Lift Mid Setpoint",  0.93, "Lift Setpoint"); 
    SmartDashboard.putData(mid);
    OperatorInterface.middleLift().onTrue(mid);

    Command high = new ApplyAdjustableSettingCommand("Lift High", "Lift High Setpoint", 1.48, "Lift Setpoint");
    SmartDashboard.putData(high);
    OperatorInterface.highLift().onTrue(high);

    // onTrue to eject? whileTrue means driver may let go too early and drop gamepiece
    OperatorInterface.eject().whileTrue(new EjectCommand(intake));
    // Assert lift is parked to take in
    OperatorInterface.intake().onTrue(new ScheduleCommand(park)
                             .andThen(new WaitUntilCommand(() -> lift.getHeight() < 0.1))
                             .andThen(new IntakeCommand(intake)));
    
    GoToNearestTagCommandHelper go = new GoToNearestTagCommandHelper(tags);
    OperatorInterface.auto_position_left().whileTrue(go.createCommand(drivetrain, false));
    OperatorInterface.auto_position_right().whileTrue(go.createCommand(drivetrain, true));

    // Smart Dashboard for lift
    nt_lift_setpoint = SmartDashboard.getEntry("Lift Setpoint");
    nt_lift_setpoint.setDefaultDouble(0.0);
  }
  
  @Override
  public void robotPeriodic()
  {
    super.robotPeriodic();

    // Set lift height from network table
    lift.setHeight(nt_lift_setpoint.getDouble(0.00));

    // Update position from cameras
    for (CameraHelper camera_helper : cameras)
      camera_helper.updatePosition(drivetrain);
  }

  @Override
  public void teleopInit()
  {
    // Bind buttons to commands
    drivetrain.setDefaultCommand(absswerve);
    // drivetrain.setDefaultCommand(absswerve);
    SwerveOI.selectRelative().onTrue(relswerve);
    SwerveOI.selectAbsolute().onTrue(absswerve);
    SwerveOI.resetDrivetrain().onTrue(new ResetHeadingCommand(drivetrain));
  }

  @Override
  public void autonomousInit()
  {
    drivetrain.setDefaultCommand(new StopCommand(drivetrain));
    autos.getSelected().schedule();
  }
}
