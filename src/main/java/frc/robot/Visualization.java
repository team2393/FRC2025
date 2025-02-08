// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Display lift etc on dashboard */
public class Visualization extends SubsystemBase
{
  private final Lift lift;
  private final MechanismLigament2d lift_mech;
  
  public Visualization(Lift lift)
  {
    this.lift = lift;

    Mechanism2d mechanism = new Mechanism2d(1.0, 1.0);
    MechanismRoot2d root = mechanism.getRoot("root1", 0, 0.1);
    root.append(new MechanismLigament2d("base", 1.0, 0, 40, new Color8Bit(Color.kBlue)));

    root = mechanism.getRoot("root2", 0.6, 0.15);
    lift_mech = new MechanismLigament2d("lift", 1, 90, 20, new Color8Bit(Color.kYellowGreen));
    root.append(lift_mech);

    SmartDashboard.putData("Visualization", mechanism);
  }

  @Override
  public void periodic()
  {
    lift_mech.setLength(0.2 + lift.getHeight());
  }
}
