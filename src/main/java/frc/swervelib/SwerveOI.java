// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.swervelib;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** OI for serving */
public class SwerveOI
{
  /** Alternate stick assignment? */
  public static final boolean ALTERNATE = false;
  
  public static final double MAX_METERS_PER_SEC = 1.0;

  public static final double MAX_ROTATION_DEG_PER_SEC = 45.0;

  public static final CommandXboxController joystick = new CommandXboxController(0);

  private static double filter(double value)
  {
    value = MathUtil.applyDeadband(value, 0.1);
    // Square
    return value * Math.abs(value);
  }

  // Dampen moves, only accelerate/break by 10 m/s
  public static SlewRateLimiter forward_slew = new SlewRateLimiter(10);
  public static SlewRateLimiter side_slew = new SlewRateLimiter(10);
  // .. and 360deg/sec
  public static SlewRateLimiter rotation_slew = new SlewRateLimiter(360);

  /** @return Forward/back speed [m/s] */
  public static double getForwardSpeed()
  {
    return forward_slew.calculate(
      MAX_METERS_PER_SEC * filter(ALTERNATE ? -joystick.getRightY() : -joystick.getLeftY()));
  }

  /** @return Left/right speed [m/s] */
  public static double getLeftSpeed()
  {
    return side_slew.calculate(
      MAX_METERS_PER_SEC * filter(ALTERNATE ? -joystick.getRightX() : -joystick.getLeftX()));
  }

  /** @return Rotational speed [deg/s] */
  public static double getRotationSpeed()
  {
    return rotation_slew.calculate(
      MAX_ROTATION_DEG_PER_SEC * filter(ALTERNATE ? -joystick.getLeftX() : -joystick.getRightX()));
  }

  /** Reset slew limiters */
  public static void reset()
  {
    forward_slew.reset(0);
    side_slew.reset(0);
    rotation_slew.reset(0);
  }
   
  public static Trigger resetDrivetrain()
  {
    return joystick.start();
  }

  public static Trigger selectRelative()
  {
    return joystick.rightBumper();
  }

  public static Trigger selectAbsolute()
  {
    return joystick.leftBumper();
  }
}
