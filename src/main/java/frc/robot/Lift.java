// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Lift that moves up/down
 * 
 *  When unpowered, lift settles down at bottom because of its weight.
 *  We read that zero position from the encoder when first enabled.
 *  During tests, first enablement is likely teleop.
 *  In competition, first enablement is auto, and by the time
 *  we enable teleop the lift may already have moved up.
 *  Using first enablement after bootup should cover both cases.
 * 
 *  Move to requested height using feed forward and PID,
 *  except for moving to "zero" which moves to a certain height above zero
 *  and then simply lets lift settle by unpowering the motors. 
 */
public class Lift extends SubsystemBase
{
  /** Height encoder calibration */
  private static final double REVS_PER_METER =  9.87 / 0.344;

  /** Maximum permitted height */
  public static final double MAX_HEIGHT = 1.5;
  /** Height below which we let the lift settle on its own */
  private static final double SETTLE_THRESHOLD = 0.02;

  /** Voltage limit to restrict speed */
  private static final double VOLTAGE_LIMIT = 10.0;

  /** Motor controller with encoder */
  
  private TalonFX primary_motor = new TalonFX(RobotMap.LIFT1);
  
  /** Other motor */
  private TalonFX secondary_motor = new TalonFX(RobotMap.LIFT2);

  private boolean brake_state = true;

  /** Has position be calibrated? */
  private boolean calibrated = false;

  /** Raw encoder reading at bottom position */
  private double bottom_offset = 0.0;

  /** Network table entries */
  private NetworkTableEntry nt_height, nt_kg, nt_ks;

  /** PID */
  // private PIDController pid = new PIDController(17, 10, 0);
  private ProfiledPIDController pid = new ProfiledPIDController(17, 10, 0,
                                            new Constraints(4,4)); 

  private double simulated_height = 0.0;
  
  public Lift()
  {
    // Primary motor is the one we control
    primary_motor.clearStickyFaults();

    // Restrict ramp to limit acceleration
    TalonFXConfiguration config = new TalonFXConfiguration()
        .withOpenLoopRamps(new OpenLoopRampsConfigs().withVoltageOpenLoopRampPeriod(0.6));
    primary_motor.getConfigurator().apply(config);    
    primary_motor.setNeutralMode(NeutralModeValue.Brake);

    // Secondary motor is inverted, follows the primary (but invert the direction)
    secondary_motor.clearStickyFaults();
    secondary_motor.getConfigurator().apply(config);    
    secondary_motor.setNeutralMode(NeutralModeValue.Brake);
    secondary_motor.setControl(new Follower(primary_motor.getDeviceID(), true));

    nt_height = SmartDashboard.getEntry("Lift Height");
    nt_kg = SmartDashboard.getEntry("Lift kg");
    nt_ks = SmartDashboard.getEntry("Lift ks");
    nt_kg.setDefaultDouble(0.3);
    nt_ks.setDefaultDouble(0.0);
    pid.setIZone(0.03);
    pid.setTolerance(0.02);
    SmartDashboard.putData("Lift PID", pid);
  }

  private void setBrake(boolean brake)
  {
    // Runtime changes cause all motors to stutter, so only update when necessary
    if (brake != brake_state)
    {
      NeutralModeValue mode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
      primary_motor.setNeutralMode(mode);
      secondary_motor.setNeutralMode(mode);
      brake_state = brake;
    }
  }

  @Override
  public void periodic()
  {
    // Enabled for the first time, never calibrated?
    if (! calibrated  &&  RobotState.isEnabled())
    {
      // Reset encoder zero/bottom position
      bottom_offset = primary_motor.getPosition().getValueAsDouble();
      pid.reset(0);
      calibrated = true;
      System.err.println("Calibrated lift bottom at " + bottom_offset + " revs");
    }
    nt_height.setDouble(getHeight());
  }  

  /** @return Lift height in meters */
  public double getHeight()
  {
    if (RobotBase.isSimulation())
      return simulated_height;
    return -(primary_motor.getPosition().getValueAsDouble() - bottom_offset) / REVS_PER_METER;
  }

  /** @param voltage Lift voltage, positive for "up" */
  public void setVoltage(double voltage)
  {
    primary_motor.setVoltage(-voltage);
  }

  public void setHeight(double desired_height)
  {
    if (RobotBase.isSimulation())
    {
      final double adjust = 0.05;
      if (desired_height > simulated_height + adjust)
        simulated_height += adjust;
      else if (desired_height < simulated_height - adjust)
        simulated_height -= adjust;
      else
        simulated_height = desired_height;
      return;
    }

    double height = getHeight();

    // Don't run above top position
    if (desired_height >= MAX_HEIGHT)
      desired_height = MAX_HEIGHT;

    // Trying to move to bottom?
    if (desired_height <= SETTLE_THRESHOLD)
    {
      // Don't "run" into the bottom.
      // Move to lower threshold
      desired_height = SETTLE_THRESHOLD;
      // If we are within 5 cm, simply let lift settle
      if (height <= SETTLE_THRESHOLD + 0.05)
      {
        setBrake(false);
        setVoltage(0.0);
        return;
      }
      // else: Move down to SETTLE_THRESHOLD..
    }

    // Not settling to bottom but actively moving to position, so enable brake
    setBrake(true);

    // Compare w/ ElevatorFeedforward
    // kg  - Gravity gain, always applied to counteract gravity
    // ks  - Static gain, minimum voltage to get moving
    // PID - .. to correct height error
    double error = desired_height - height;
    double voltage = nt_kg.getDouble(0.0)
                   + nt_ks.getDouble(0.0) * Math.signum(error)
                   + pid.calculate(height, desired_height);
    if (voltage > VOLTAGE_LIMIT)
      voltage = VOLTAGE_LIMIT;
    else if (voltage < -VOLTAGE_LIMIT)
      voltage = -VOLTAGE_LIMIT;
    setVoltage(voltage);
  }

  public boolean isAtHight()
  {
    return pid.atGoal();
  }
}
