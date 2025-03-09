// Copyright (c) FIRST team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.swervelib.SwerveOI;

/** Operator Interface: Which stick or button does what? */
public class OperatorInterface extends SwerveOI
{
  public static final CommandXboxController joystick = new CommandXboxController(0);

  public static Trigger intake()
  {
    return joystick.a();
  }

  public static Trigger eject()
  {
    return joystick.b();
  }

  /** @return Is left trigger active? */
  public static Trigger auto_position_left()
  {
    return joystick.leftTrigger();
  }

  /** @return Is right trigger active? */
  public static Trigger auto_position_right()
  {
    return joystick.rightTrigger();
  }
}
