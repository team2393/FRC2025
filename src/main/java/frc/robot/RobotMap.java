// Copyright (c) FIRST team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Robot (Hardware) Map: What's connected where? */
public class RobotMap
{
  // Use comments for port and fuse on the power distribution port
  // Constants define CAN IDs or RoboRIO ports

  // RoboRIO: 10 Amp, port 21
  // Radio  : 10 Amp, port 20

  // SparkMax(?) drive motors: 40 Amp Fuse
  public static final int FRONT_LEFT_DRIVE  = 9;   // Port 10
  public static final int FRONT_RIGHT_DRIVE = 8;   // Port 11
  public static final int BACK_RIGHT_DRIVE  = 2;   // Port 12
  public static final int BACK_LEFT_DRIVE   = 3;   // Port 13

  // SparkMax(?) rotator motors: 40 Amp Fuse
  public static final int FRONT_LEFT_ROTATE  = 7;   // Port 9
  public static final int FRONT_RIGHT_ROTATE = 10;  // Port 8
  public static final int BACK_RIGHT_ROTATE  = 4;   // Port 7
  public static final int BACK_LEFT_ROTATE   = 1;   // Port 6

  // CANCoder(?) angle sensors: 10 Amp Fuse
  public static final int FRONT_LEFT_ANGLE  = 11;   // Port 3
  public static final int FRONT_RIGHT_ANGLE = 12;   // Port 2
  public static final int BACK_RIGHT_ANGLE  = 6;    // Port 1
  public static final int BACK_LEFT_ANGLE   = 5;    // Port 0
}
