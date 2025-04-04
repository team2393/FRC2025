// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.led;

import edu.wpi.first.wpilibj2.command.Command;

/** Command that sets all colors of LEDRing to red and is then finished */
public class SetToRed extends Command
{
  private LEDRing ring;

  public SetToRed(LEDRing the_ring)
  {
    ring = the_ring; 
    addRequirements(ring);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    for (int i=0;  i<LEDRing.N;  ++i)
      ring.buffer.setRGB(i, 255, 0, 0);
  
    ring.led.setData(ring.buffer);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return true;
  }
}
