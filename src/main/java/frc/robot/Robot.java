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
  private final PowerDistribution power = new PowerDistribution();

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
    SwerveOI.MAX_METERS_PER_SEC = SwerveDrivetrain.MAX_METERS_PER_SEC = 3.0; // 1 .. 3
    SwerveOI.MAX_ROTATION_DEG_PER_SEC = SwerveDrivetrain.MAX_ROTATION_DEG_PER_SEC = 270;
    SwerveOI.forward_slew = new SlewRateLimiter(3.0);
    SwerveOI.side_slew = new SlewRateLimiter(3.0);
    SwerveOI.rotation_slew = new SlewRateLimiter(360);
    AutoTools.config = new TrajectoryConfig(4, 4);
    SwerveToPositionCommand.MAX_SPEED = 5;

    power.clearStickyFaults();
    power.resetTotalEnergy();
    // TODO Display PD on Dashboard?
    // This results in frequent messages because PD is slow to update
    // SmartDashboard.putData("PowerPanel", power);

    OperatorInterface.reset();
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

    OperatorInterface.eject().whileTrue(new EjectCommand(intake));
    OperatorInterface.reverse().whileTrue(new ReverseCommand(intake));
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

  private void updateDriveMode()
  {
    // Change default, cancel the 'wrong' mode if it's running,
    // but otherwise don't schedule abs nor rel because
    // we might right now be running an auto-position command
    if (OperatorInterface.absoluteModeSwitch().getAsBoolean())
    {
      if (drivetrain.getDefaultCommand() != absswerve)
      {
        System.out.println("ABSOLUTE MODE");
        drivetrain.setDefaultCommand(absswerve);
        if (relswerve.isScheduled())
          relswerve.cancel();
      }
    }
    else
      if (drivetrain.getDefaultCommand() != relswerve)
      {
        System.out.println("RELATIVE MODE");
        drivetrain.setDefaultCommand(relswerve);
        if (absswerve.isScheduled())
          absswerve.cancel();
      }
  }

  @Override
  public void teleopInit()
  {
    updateDriveMode();
    // Bind buttons to commands
    SwerveOI.selectRelative().onTrue(relswerve);
    SwerveOI.selectAbsolute().onTrue(absswerve);
    SwerveOI.resetDrivetrain().onTrue(new ResetHeadingCommand(drivetrain));
  }

  @Override
  public void teleopPeriodic()
  {
    updateDriveMode();
  }

  @Override
  public void autonomousInit()
  {
    drivetrain.setDefaultCommand(new StopCommand(drivetrain));
    autos.getSelected().schedule();
  }
}
