// Copyright (c) FIRST team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Robot (Hardware) Map: What's connected where? */
public class RobotMap
{
  // Use comments for port and fuse on the power distribution panel
  // Constants define CAN IDs or RoboRIO ports

  // RoboRIO: 10 Amp, port 21
  // Radio  : 10 Amp, port 20

  // Kraken drive motors: 40 Amp Fuse
  public static final int FRONT_LEFT_DRIVE  = 3;   // Port  11
  public static final int FRONT_RIGHT_DRIVE = 7;   // Port 2
  public static final int BACK_RIGHT_DRIVE  = 6;   // Port 6
  public static final int BACK_LEFT_DRIVE   = 2;   // Port 7

  // Kraken rotator motors: 40 Amp Fuse
  public static final int FRONT_LEFT_ROTATE  = 1;   // Port 9
  public static final int FRONT_RIGHT_ROTATE = 5;   // Port 4
  public static final int BACK_RIGHT_ROTATE  = 8;   // Port 8
  public static final int BACK_LEFT_ROTATE   = 4;   // Port 5

  // CANCoder angle sensors: 10 Amp Fuse
  public static final int FRONT_LEFT_ANGLE  = 17;    // Port 15
  public static final int FRONT_RIGHT_ANGLE = 57;    // Port 15
  public static final int BACK_RIGHT_ANGLE  = 23;    // Port 15
  public static final int BACK_LEFT_ANGLE   = 24;    // Port 15

  // Lift Kraken motors
  public static final int LIFT1 = 9;
  public static final int LIFT2 = 10;

  // Intake?
  public static final int INTAKE = 11;

  // Arm?
  public static final int ARM = 12;

  // DIGITAL IO
  public static final int INTAKE_SENSOR = 0;
  public static final int ARM_ENCODER = 1;
}
