package org.usfirst.frc.team2607.robot.auto;

import java.util.ArrayList;

import org.usfirst.frc.team2607.robot.Constants;
import org.usfirst.frc.team2607.robot.Robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * @author Cerora
 *
 */
public class AutonomousManager {
	  
	Robot robot;
	public ArrayList<AutonomousMode> modes = new ArrayList<AutonomousMode>();

	AutonomousManager(Robot robot){
		this.robot = robot;
		
		// add Autonomous modes to the list here
		modes.add(new DoNothingFailsafe(robot));
		modes.add(new CenterRightLeftSwitch(robot));
		modes.add(new RScaleSideLeftSide(robot));
		modes.add(new LeftCrossLine(robot));
		modes.add(new RightCrossLine(robot));
		modes.add(new DriveStraightTest(robot));
		modes.add(new DoNothing(robot));
		
		
	}
	
	public AutonomousMode getModeByName (String name){
		for (AutonomousMode m : modes){
			if (m.getName().equals(name))
				return m;
		}
		
		try {
			throw new Exception();
		} catch (Exception e) {
			System.err.println("Mode not found");
			e.printStackTrace();
			return new DoNothingFailsafe(robot);
		}
	}
	
	public AutonomousMode getModeByIndex (int index){
		try {
			return modes.get(index);
		} catch (IndexOutOfBoundsException e){
			System.err.println("Mode out of array bounds");
			e.printStackTrace();
			return new DoNothingFailsafe(robot);
		}
	}
			
}