package org.usfirst.frc.team2607.robot;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
		public class SwitchReader
		{
			DigitalInput limitSwitch;
			SwitchReader()
			{
		    	limitSwitch = new DigitalInput(1);
		    }
		    public void operatorControl()
		    {
		    	while (limitSwitch.get()) 
		    	{
		    		Timer.delay(10);
		    	}
		       
		    }
		    public boolean isSwitchOpen()
		    {
		    	return !limitSwitch.get();
		    }
		}

	
