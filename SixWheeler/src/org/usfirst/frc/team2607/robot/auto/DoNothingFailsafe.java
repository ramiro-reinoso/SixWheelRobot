package org.usfirst.frc.team2607.robot.auto;

import org.usfirst.frc.team2607.robot.Robot;

public class DoNothingFailsafe extends AutonomousMode {

	Robot theRobot;
	
	public DoNothingFailsafe(Robot r) {
		theRobot = r;
	}
	
	@Override
	public void run() {
		
		System.out.println("DoNothingFailSafe - not doing anything :-)");

	}

	@Override
	public String getName() {
		
		return "Do Nothing - Fail Safe";
	}

}
