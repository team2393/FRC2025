// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.demo;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OperatorInterface;
import frc.tools.CommandRobotBase;

/** SparkMax Motor Demo */
public class MotorSparkMaxDemoRobot extends CommandRobotBase
{
    // Move motor by hand, note distance in meters over indicated turns 
    private final static double METERS_PER_TURN = 1.0/1.0;
    
    private final SparkMax motor = new SparkMax(1, MotorType.kBrushless);

    public MotorSparkMaxDemoRobot()
    {
        motor.clearFaults();
        // Some settings are only available via "...Configurator" class
        motor.configure(new SparkMaxConfig().idleMode(IdleMode.kCoast)
                                            .openLoopRampRate(0.1),
                        // Suggested in https://github.com/wpilibsuite/2025Beta/discussions/27 :
                        // 'Reset' to reset all but CAN ID, Motor Type, Idle Mode(!),
                        // then apply what's specified: idle mode, ramp rate
                        ResetMode.kNoResetSafeParameters,
                        // Don't 'persist' because that is slow
                        PersistMode.kNoPersistParameters);
        
        // Others via methods
        motor.setInverted(false);
    }

    @Override
    public void robotPeriodic()
    {
        super.robotPeriodic();
        SparkAbsoluteEncoder encoder = motor.getAbsoluteEncoder();
        double turns = encoder.getPosition();
        SmartDashboard.putNumber("Turns", turns);
        SmartDashboard.putNumber("Position", turns * METERS_PER_TURN);
        SmartDashboard.putNumber("Speed", encoder.getVelocity() * METERS_PER_TURN);
    }    

    @Override
    public void teleopPeriodic()
    {
        motor.setVoltage(OperatorInterface.joystick.getRightY()*12.0);
    }
}
