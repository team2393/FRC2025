package frc.robot;

import com.ctre.phoenix6.Orchestra;

public class Music
{
    Orchestra music;
    
    public Music()
    {
        music = new Orchestra("src/deploy/music.chrp");
        
        // TODO Add motors as "instruments
        // music.addInstrument(.. some falcon motor from drivetrain, lift, ...);
    }

    public void play()
    {
        music.play();
    }
}
