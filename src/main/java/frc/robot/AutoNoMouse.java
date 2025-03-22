// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import static frc.tools.AutoTools.createTrajectory;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.swervelib.ResetPositionCommand;
import frc.swervelib.SelectAbsoluteTrajectoryCommand;
import frc.swervelib.SelectRelativeTrajectoryCommand;
import frc.swervelib.RotateToHeadingCommand;
import frc.swervelib.SwerveDrivetrain;
import frc.swervelib.SwerveToPositionCommand;
import frc.swervelib.VariableWaitCommand;
\import frc.tools.AutoTools;
import frc.tools.SequenceWithStart;

/** Auto-no-mouse routines */
public class AutoNoMouse
{
  /** Create all our auto-no-mouse commands 
 * @param intake 
   * @param lift */
  public static List<Command> createAutoCommands(SwerveDrivetrain drivetrain, AprilTagFieldLayout tags, Intake intake, Lift lift)
  {
    // List of all autonomouse commands
    final List<Command> autos = new ArrayList<>();

    // Each auto is created within a { .. block .. } so we get local variables for 'path' and the like.
    // Each auto should start with a VariableWaitCommand to allow coordination with other teams

    { // Drive forward 0.5 m using a (simple) trajectory
      SequentialCommandGroup auto = new SequentialCommandGroup();
      auto.setName("Forward 0.5m");
      auto.addCommands(new VariableWaitCommand());
      auto.addCommands(new SelectRelativeTrajectoryCommand(drivetrain));
      Trajectory path = createTrajectory(true, 0, 0, 0,
                                            0.50, 0, 0);
      auto.addCommands(drivetrain.followTrajectory(path, 0));
      autos.add(auto);
    }

    for (String level : List.of("Low", "Mid", "High"))
      for (boolean right : List.of(false, true))
      { // Drive forward 0.5 m, then move to low/mid/high, left/right reef and drop
        SequentialCommandGroup auto = new SequentialCommandGroup();
        auto.setName("0.5m,aim,drop " + level.toLowerCase() + (right ? " right" : " left"));
        auto.addCommands(new VariableWaitCommand());
        // Move 0.5m
        auto.addCommands(new SelectRelativeTrajectoryCommand(drivetrain));
        Trajectory path = createTrajectory(true, 0, 0, 0,
                                              0.50, 0, 0);
        auto.addCommands(drivetrain.followTrajectory(path, 0).asProxy());
        auto.addCommands(new SelectAbsoluteTrajectoryCommand(drivetrain));
        
        // Wait for camera to acquire position
        auto.addCommands(new WaitCommand(5));

        auto.addCommands(new GoToNearestTagCommandHelper(tags).createCommand(drivetrain, right));

        auto.addCommands(new InstantCommand(() ->
        { // Set "Lift Setpoint" to value of "Lift Low/Mid/High Setpoint"
          double height = SmartDashboard.getNumber("Lift " + level + " Setpoint", 0);
          SmartDashboard.putNumber("Lift Setpoint", height);
        }));
        // Wait for lift to be at commanded height
        auto.addCommands(new WaitUntilCommand(lift::isAtHight));

        // Hope this works out...
        auto.addCommands(new EjectCommand(intake));

        autos.add(auto);
      }

    { // Drive forward and back 1.5 m using a (simple) trajectory
      SequentialCommandGroup auto = new SequentialCommandGroup();
      auto.setName("2(Forward & Back) 1.5m");
      auto.addCommands(new VariableWaitCommand());
      auto.addCommands(new SelectRelativeTrajectoryCommand(drivetrain));
      Trajectory path = createTrajectory(true, 0, 0, 0,
                                            1.50, 0, 0);
      auto.addCommands(drivetrain.followTrajectory(path, 0).asProxy());

      auto.addCommands(new WaitCommand(2));

      path = createTrajectory(true, 1.50, 0, 180,
                                       0, 0, 180);
      auto.addCommands(drivetrain.followTrajectory(path, 0).asProxy());
      path = createTrajectory(true, 0, 0, 0,
                                 1.50, 0, 0);
      auto.addCommands(drivetrain.followTrajectory(path, 0).asProxy());
      path = createTrajectory(true, 1.50, 0, 180,
                                       0, 0, 180);
      auto.addCommands(drivetrain.followTrajectory(path, 0).asProxy());
      autos.add(auto);
    }

    { // Triangle course with SwerveToPositionCommand
      SequentialCommandGroup auto = new SequentialCommandGroup();
      Timer timer = new Timer();
      auto.setName("Trig Points");
      auto.addCommands(new VariableWaitCommand());
      auto.addCommands(new InstantCommand(() -> timer.restart()));
      // SwerveToPositionCommand is always absolute,
      // so reset position to zero
      auto.addCommands(new ResetPositionCommand(drivetrain));
      auto.addCommands(new SwerveToPositionCommand(drivetrain, 2.0, 0.0));
      auto.addCommands(new SwerveToPositionCommand(drivetrain, 1.0, 0.5));
      auto.addCommands(new SwerveToPositionCommand(drivetrain, 0.0, 0.0));
      auto.addCommands(new InstantCommand(() -> System.out.printf("Time: %.1f sec\n", timer.get())));
      autos.add(auto);
    }

    { // Triangle course with trajectory
      SequentialCommandGroup auto = new SequentialCommandGroup();
      Timer timer = new Timer();
      auto.setName("Trig Traj");
      auto.addCommands(new VariableWaitCommand());
      auto.addCommands(new InstantCommand(() -> timer.restart()));
      // Trajectory can be relative to current position
      auto.addCommands(new SelectRelativeTrajectoryCommand(drivetrain));
      Trajectory path = createTrajectory(true, 0.0, 0.0,   0.0,
                                               2.0, 0.0,  90.0,
                                               1.0, 0.5, 180.0,
                                               0.0, 0.0, 180.0);
      auto.addCommands(drivetrain.followTrajectory(path, 0));
      auto.addCommands(new InstantCommand(() -> System.out.printf("Time: %.1f sec\n", timer.get())));
      autos.add(auto);
    }

    { // Drive a 1.5 square using SwerveToPositionCommand & RotateToHeadingCommand
      SequentialCommandGroup auto = new SequentialCommandGroup();
      auto.setName("1.5m Square (rot)");
      auto.addCommands(new VariableWaitCommand());

      // SwerveToPositionCommand & RotateToHeadingCommand are always absolute,
      // so reset position to zero
      auto.addCommands(new ResetPositionCommand(drivetrain));

      auto.addCommands(new SwerveToPositionCommand(drivetrain, 1.5, 0.0));
      auto.addCommands(new RotateToHeadingCommand(drivetrain, 90));

      auto.addCommands(new SwerveToPositionCommand(drivetrain, 1.5, 1.5));
      auto.addCommands(new RotateToHeadingCommand(drivetrain, 180));

      auto.addCommands(new SwerveToPositionCommand(drivetrain, 0.0, 1.5));
      auto.addCommands(new RotateToHeadingCommand(drivetrain, -90));

      auto.addCommands(new SwerveToPositionCommand(drivetrain, 0.0, 0.0));
      auto.addCommands(new RotateToHeadingCommand(drivetrain, 0));

      autos.add(auto);
    }

    { // Start with preloaded coral, drop, get another, drop, ...
      // like https://www.chiefdelphi.com/t/kitbot-4-coral-level-1-auto/479325
      SequentialCommandGroup auto = new SequenceWithStart("CoralRun", 8.02, 5.41, 180);
      auto.addCommands(new VariableWaitCommand());
      auto.addCommands(new SelectAbsoluteTrajectoryCommand(drivetrain, 8.02, 5.41, 180));
      Timer timer = new Timer();
      auto.setName("CoralRun");
      auto.addCommands(new VariableWaitCommand());
      auto.addCommands(new InstantCommand(() -> timer.restart()));
      // Drive from start positon to 1st drop
      Trajectory path = createTrajectory(true, 8.02, 5.41, 180.0,
                                               6.12, 4.01, 180.0);
      auto.addCommands(drivetrain.followTrajectory(path, 180));
      auto.addCommands(new PrintCommand("Drop pre-loaded coral"));
      auto.addCommands(new WaitCommand(1.0));
      // Pickup 2nd
      auto.addCommands(new PrintCommand("Open Intake"));
      path = createTrajectory(true, 6.12, 4.01,  -80,
                                    5.8,  1.6,  -140,
                                    1.09, 1.02, -170);
      auto.addCommands(drivetrain.followTrajectory(path, 50));
      auto.addCommands(new PrintCommand("Close Intake"));
      // Drop 2nd
      path = createTrajectory(true, 1.09, 1.02, 20,
                                    3.7, 2.6,  60);
      auto.addCommands(drivetrain.followTrajectory(path, 60));
      auto.addCommands(new PrintCommand("Drop 2nd coral"));
      auto.addCommands(new WaitCommand(1.0));
      // Pickup 3rd
      auto.addCommands(new PrintCommand("Open Intake"));
      path = createTrajectory(true, 3.7, 2.6,  -120,
                                    1.09, 1.02, -130);
      auto.addCommands(drivetrain.followTrajectory(path, 50));
      auto.addCommands(new PrintCommand("Close Intake"));
      // Drop 3nd
      path = createTrajectory(true, 1.09, 1.02, 20,
                                    2.6,  2.2, 90,
                                    2.8,  4.0, 45);
      auto.addCommands(drivetrain.followTrajectory(path, 0));
      auto.addCommands(new PrintCommand("Drop 3nd coral"));
      auto.addCommands(new WaitCommand(1.0));

      auto.addCommands(new InstantCommand(() -> System.out.printf("Time: %.1f sec\n", timer.get())));
      autos.add(auto);
    }
    {
      SequentialCommandGroup auto = new SequenceWithStart("CoralRunRed", 9.355, 1.3, 0);
      auto.addCommands(new VariableWaitCommand());
      auto.addCommands(new SelectAbsoluteTrajectoryCommand(drivetrain, 9.355, 1.3, 0));

      Trajectory path = createTrajectory(true, 9.355, 1.3, 0,
                                                                11.55, 4, 0 );
      auto.addCommands(drivetrain.followTrajectory(path, 0));
      System.out.println("Drop Coral"
                          + "\n Open Intake");
      path = createTrajectory(true, 11.55,6,0,
                                                16,7.1, 25);
      auto.addCommands(drivetrain.followTrajectory(path, 25));
      
      autos.add(auto); 
    }
    {
      SequentialCommandGroup auto = new SequenceWithStart("CoralRunCircle", 9.5, 2.9, 0);
      auto.addCommands(new VariableWaitCommand());
      auto.addCommands(new SelectAbsoluteTrajectoryCommand(drivetrain, 9.5, 2.9, 0));
      auto.addCommands(AutoTools.followPathWeaver(drivetrain, "CoralCircle", 0));
      autos.add(auto);
    }

    return autos;
  }
}
