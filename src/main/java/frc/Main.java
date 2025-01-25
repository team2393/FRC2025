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
    // RobotBase.startRobot(frc.demo.FirstRobotDemo::new);
    // RobotBase.startRobot(frc.demo.MotorPhoenix6DemoRobot::new);
    // RobotBase.startRobot(() -> new frc.swervelib.DriverDemoRobot(new frc.swervebot.Driver(0)));
    // RobotBase.startRobot(() -> new frc.swervelib.RotatorDemoRobot(new frc.swervebot.Rotator(0, 0)));
    
    // TODO: Robot drivetrain
    // RobotBase.startRobot(() -> new frc.swervelib.DriverDemoRobot(new frc.robot.Driver(0, 10)));
    // RobotBase.startRobot(() -> new frc.swervelib.RotatorDemoRobot(new frc.robot.Rotator(0, 11, 11, 0.0)));

    // RobotBase.startRobot(() ->
    //   new frc.swervelib.SwerveModuleDemoRobot(
    //     new SwerveModule[]
    //     {
    //       new frc.swervelib.SwerveModule(new frc.swervebot.Rotator(0,  -17), new frc.swervebot.Driver(0)),
    //       new frc.swervelib.SwerveModule(new frc.swervebot.Rotator(1,  -89+180), new frc.swervebot.Driver(1)),
    //       new frc.swervelib.SwerveModule(new frc.swervebot.Rotator(2,   20.8+180), new frc.swervebot.Driver(2)),
    //       new frc.swervelib.SwerveModule(new frc.swervebot.Rotator(3, -106), new frc.swervebot.Driver(3))
    //     }));

    // RobotBase.startRobot(frc.led.LEDRingDemoRobot::new);

    // RobotBase.startRobot(frc.swervebot.SwerveBot::new);
    RobotBase.startRobot(frc.robot.Robot::new);
  }
}
