package org.usfirst.frc.team2607.robot.auto;

import org.usfirst.frc.team2607.robot.Robot;

public class DoNothing extends AutonomousMode {

	Robot theRobot;
	
	public DoNothing(Robot r) {
		theRobot = r;
	}
	
	@Override
	public void run() {
		
		System.out.println("DoNothing - not doing anything :-)");

	}

	@Override
	public String getName() {
		
		return "Do Nothing";
	}

}
