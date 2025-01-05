// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Shooter Arm: Motor to rotate shooter up/down
 * 
 *  REV Through Bore encoder connected to Spark Max via
 *  Absolute Encoder Adapter    REV-11-3326 
 */
public class Arm extends SubsystemBase
{
  private SparkMax motor = new SparkMax(RobotMap.ARM, MotorType.kBrushless);
  /** Through Bore Encoder to measure angle.
   *  'A'/'S' switch on side of encoder must be in 'A' position.
   *  Absolute readout can use (white, red, black) into DI,
   *  but we have it plugged into the motor's SparkMax
   */
  private final SparkAbsoluteEncoder encoder = motor.getAbsoluteEncoder();
  // private final DutyCycleEncoder encoder = new DutyCycleEncoder(RobotMap.ARM_ENCODER);

  /** Zero degrees = arm horizontally out */
  private final static double ZERO_OFFSET = 0;
 
  // Rotate at a max speed of 45 deg/sec
  private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(45, 45);
  private ProfiledPIDController pid = new ProfiledPIDController(0.3, 0.03, 0.01, constraints);
  private NetworkTableEntry nt_kg, nt_angle, nt_desired_angle;
  private boolean isDone = false;
  
  public Arm()
  {
    motor.clearFaults();
    motor.configure(new SparkMaxConfig().idleMode(IdleMode.kCoast)
                                        .openLoopRampRate(0.1)
                                        .inverted(false),
                    ResetMode.kNoResetSafeParameters,
                    PersistMode.kNoPersistParameters);

    reset();

    nt_kg = SmartDashboard.getEntry("Arm kg");
    nt_kg.setDefaultDouble(0.25);
    nt_angle = SmartDashboard.getEntry("Arm Angle");
    nt_desired_angle = SmartDashboard.getEntry("Set Arm Angle");
    nt_desired_angle.setDefaultDouble(55);
    SmartDashboard.putData("Arm PID", pid);
  }

  public void reset()
  {
    pid.reset(getAngle());
  }

  public double getAngle()
  {
    return -encoder.getPosition()*360 - ZERO_OFFSET;
  }

  public void setAngle(double degrees)
  {
    nt_desired_angle.setNumber(degrees);
    isDone = false;
  }

  public boolean atDesiredAngle()
  {
    return isDone;
  }

  @Override
  public void periodic()
  {
    final double angle = getAngle();
    nt_angle.setDouble(angle);

    double kg = nt_kg.getDouble(0.25);
    double setpoint = MathUtil.clamp(nt_desired_angle.getDouble(50), 20, 60);

    double voltage = kg * Math.cos(Math.toRadians(angle))
                   + pid.calculate(angle, setpoint);
    motor.setVoltage(voltage);

    isDone = Math.abs(angle - setpoint) < 1;
  }
}