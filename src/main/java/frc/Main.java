// Copyright (c) FIRST team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc;

import edu.wpi.first.wpilibj.RobotBase;

/** Java 'Main'. Modify this file to select which robot to run */
public final class Main
{
  public static void main(String... args)
  {
    // System.out.println("Hello, robot!");
    // RobotBase.startRobot(frc.tools.CommandRobotBase::new);
    // RobotBase.startRobot(frc.demo.MotorPhoenix6DemoRobot::new);
    RobotBase.startRobot(() -> new frc.swervelib.DriverDemoRobot(new frc.swervebot.Driver(0)));
    // RobotBase.startRobot(() -> new frc.swervelib.RotatorDemoRobot(new frc.swervebot.Rotator(0, 0)));
    // RobotBase.startRobot(frc.swervebot.SwerveBot::new);
  }
}
