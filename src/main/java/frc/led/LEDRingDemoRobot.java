// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.led;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.tools.CommandRobotBase;

public class LEDRingDemoRobot extends CommandRobotBase
{
  private final LEDRing ring = new LEDRing();

  public LEDRingDemoRobot()
  {
    ring.setDefaultCommand(new Comet(ring));
    // ring.setDefaultCommand(new ColorPair(ring, ));
  }

  @Override
  public void teleopInit()
  {
    // Roll red and green around the ring for 3 seconds
    Command roll = new ParallelRaceGroup(
                      new ColorPair(ring, Color.kRed, Color.kGreen),
                      new WaitCommand(3)
                    );

    // Alternate so called "fluent" way of getting the same end result, different colors:
    Command roll2 = new ColorPair(ring, Color.kDarkGreen, Color.kDarkGoldenrod).withTimeout(3);

    // Blink red/green a few times
    Command blink = new SequentialCommandGroup(
                          new SetToRed(ring),
                          new WaitCommand(0.5),
                          new SetToGreen(ring),
                          new WaitCommand(0.5),
                          new SetToRed(ring),
                          new WaitCommand(0.5),
                          new SetToGreen(ring),
                          new WaitCommand(0.5),
                          new SetToRed(ring),
                          new WaitCommand(0.5),
                          new SetToGreen(ring),
                          new WaitCommand(0.5)
                        );

      // Again alternate way:
      // blink = new SetToRed(ring)
      //     .andThen(Commands.waitSeconds(0.5))
      //     .andThen(new SetToGreen(ring))
      //     .andThen(Commands.waitSeconds(0.5))
      //     .andThen(new SetToRed(ring))
      //     .andThen(Commands.waitSeconds(0.5))
      //     .andThen(new SetToGreen(ring))
      //     .andThen(Commands.waitSeconds(0.5));

    // Keep doing those two patterns, one after the other    
    new RepeatCommand(new SequentialCommandGroup(roll, roll2, blink)).schedule();
    // .. or ...
    // roll.andThen(blink).repeatedly().schedule();
  }
}
