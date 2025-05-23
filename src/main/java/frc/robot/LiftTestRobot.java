// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.tools.CommandRobotBase;

/** Lift test robot
 *
 * Setup Procedure:
 * - Use correct motors and sensor types, ID, ..
 * 
 * Teleop, right stick:
 * - Disconnect both motors' power wires from speed controllers.
 *   (Can't do that for integrated motor/controller? Then remove a gear?)
 * - After bootup, briefly enable teleop. This first enablement
 *   should zero the height reading.
 * - Enable teleop. Check if moving 'up' with positive voltage
 *   indeed moves primary motor 'up'.
 *   If not, switch motor inversion.
 * - Connect secondary motor to speed controller and check that it moves
 *   in correct direction with primary. If not, update its inversion setting.
 * - Calibrate height encoder, set REVS_PER_METER and MAX_HEIGHT.
 * - Connect power wires for primary motor.
 * 
 * Auto-no-mouse:
 * - Start with "Setpoint" = 0.0, assert that motors are not powered.
 * - Enter negative setpoint, assert that motors are not powered.
 * - Disable, move lift halfway up, enter halfway Setpoint, enable auto.
 *   Adjust kg such that motors counteract gravity and lift stays put.
 * - Enter a setpoint above current height (or let lift settle below setpoint)
 *   and adjust ks such that motors just barely start moving 'up'.
 * - Enter different setpoints and adjust P(ID) such that lift gets there.
 * 
 * .. only apply ks when moving up, since motors need no get-going voltage
 *    to move down?
 * .. is brake necessary, or is there always a holding voltage so brake
 *    never active anyway?
 * .. then update to ProfiledPID?
 */
public class LiftTestRobot extends CommandRobotBase
{
  private final Lift lift = new Lift();
  
  public LiftTestRobot()
  {
    new Visualization(lift);
    SmartDashboard.setDefaultNumber("Setpoint1", 0.0);
    SmartDashboard.setDefaultNumber("Setpoint2", 0.0);
  }

  @Override
  public void teleopPeriodic()
  {
    // For 'up', send positive voltage
    double voltage = -3.0 * OperatorInterface.joystick.getRightY();
    if (lift.getHeight() >= Lift.MAX_HEIGHT   &&  voltage > 0)
      voltage = 0;
    lift.setVoltage(voltage);
    SmartDashboard.putNumber("Lift Voltage", voltage);
  }

  @Override
  public void autonomousPeriodic()
  {
    double height = (System.currentTimeMillis() / 5000) % 2 == 1
                  ? SmartDashboard.getNumber("Setpoint1", 0)
                  : SmartDashboard.getNumber("Setpoint2", 0);
    lift.setHeight(height);
  }
}