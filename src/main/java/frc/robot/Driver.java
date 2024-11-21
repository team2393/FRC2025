// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.swervelib.DriverBase;

/** Driver using Kraken */
public class Driver extends DriverBase
{
  // TODO Calibrate
  private final static double METERS_PER_TURN = 1.0;

  private final TalonFX motor;

  /** @param index Driver index 0..3
   *  @param id CAN id
   */
  public Driver(int index, int id)
  {
    // TODO Find PID settings
    super(index, 0.05, 2.27, 1, 1, 0);
    motor = new TalonFX(id);
    TalonFXConfiguration config = new TalonFXConfiguration()
        .withOpenLoopRamps(new OpenLoopRampsConfigs().withVoltageOpenLoopRampPeriod(0.3));
    motor.getConfigurator().apply(config);    
    motor.clearStickyFaults();
    motor.setNeutralMode(NeutralModeValue.Brake);
  }
  
  protected double getRawPosition()
  {
    return motor.getPosition().getValueAsDouble() * METERS_PER_TURN;
  }

  protected double getRealSpeed()
  {
    return motor.getVelocity().getValueAsDouble() * METERS_PER_TURN;
  }

  public void setVoltage(double voltage)
  {
    motor.setVoltage(voltage);
  }
}