// Copyright (c) FIRST team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
// TODO import frc.swervelib.SwerveOI;

/** Operator Interface: Which stick or button does what? */
public class OperatorInterface // TODO extends SwerveOI
{
  public static final CommandXboxController joystick = new CommandXboxController(0);

  public static Trigger fire()
  {
    return joystick.y();
  }

  public static Trigger resetHeading()
  {
    return joystick.rightBumper();
  }
}
