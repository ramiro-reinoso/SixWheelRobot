package org.usfirst.frc.team2607.robot.auto;

import org.usfirst.frc.team2607.robot.Constants;
import org.usfirst.frc.team2607.robot.ReadTrajectoryPoints;
import org.usfirst.frc.team2607.robot.Robot;

import com.ctre.phoenix.motion.SetValueMotionProfile;
import com.ctre.phoenix.motion.TrajectoryPoint;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;

public class CenterRightLeftSwitch extends AutonomousMode {
	
	Robot theRobot ;
	public ReadTrajectoryPoints trajPointsLeft, trajPointsRight;
	public TrajectoryPoint point = new TrajectoryPoint();
	private int pointsCount = 0;
	private int pointsCountRight = 0;
	private int pointsCountLeft = 0;
	public boolean isRotating = false;
	
	public CenterRightLeftSwitch (Robot r) {
		theRobot = r;
	}
	

	
	
	public void startLoadingPoints () {
		
		theRobot.leftTrans.setCoast();
		theRobot.rightTrans.setCoast();
		// start MP execution
        theRobot.leftTrans.setMotorModeMotionProfile(SetValueMotionProfile.Disable);
        theRobot.rightTrans.setMotorModeMotionProfile(SetValueMotionProfile.Disable);
        
        theRobot.periodicProcess.startPeriodic(.005);
		
		theRobot.leftTrans.setInverted(false);
		theRobot.leftTrans.setEncoderPhase(false);
		
		//theRobot.elevator.setPosition(18000); // practice 18000 comp 20500
		
		double [] tempPoint = new double [3];
		
		theRobot.leftTrans.zeroEncoder();
		theRobot.rightTrans.zeroEncoder();
		
		theRobot.leftTrans.setHighGear(false, true);
		theRobot.rightTrans.setHighGear(false, true);
	
		theRobot.leftTrans.setClearTrajectoryBuffer();
		theRobot.rightTrans.setClearTrajectoryBuffer();
		
		theRobot.leftTrans.setClearHasUnderrun();
		theRobot.rightTrans.setClearHasUnderrun();
	
		String gameData;
		gameData = DriverStation.getInstance().getGameSpecificMessage();
        if(gameData.length() > 0)
        {
            if(gameData.charAt(0) == 'L')
            {
            	trajPointsRight = new ReadTrajectoryPoints();
            	trajPointsRight.addTrajectoryPoints("rw_traj_LeftSwitch_ticks.csv",1.14);
            	trajPointsLeft = new ReadTrajectoryPoints();
            	trajPointsLeft.addTrajectoryPoints("lw_traj_LeftSwitch_ticks.csv",1.14);
            } else {
            	trajPointsRight = new ReadTrajectoryPoints();
            	trajPointsRight.addTrajectoryPoints("rw_traj_RightSwitch_ticks.csv",1.14);
            	trajPointsLeft = new ReadTrajectoryPoints();
            	trajPointsLeft.addTrajectoryPoints("lw_traj_RightSwitch_ticks.csv",1.14);
            }
        }
    
        point.timeDur = TrajectoryPoint.TrajectoryDuration.Trajectory_Duration_10ms;
		
        pointsCount = 0;
	
	/*
	 * The loop before loads trajectory points to the top buffer of both transmissions, the right and the left side
	 * transmissions.  It will load points to the top buffer until it either loads all the points in the trajectory
	 * or until it fills the top buffer.  The maximum size of the top buffer (in trajectory points) is defined in
	 * Constant.kMaxTopBuffer.
	 */
	
     while ((pointsCount < trajPointsLeft.getTotalPoints()) && (theRobot.leftTrans.getTopBufferCount() < Constants.kMaxTopBuffer)
    		 && theRobot.leftTrans.getBtmBufferCount() < 100) {
		
        	tempPoint = trajPointsLeft.getNextTrajPoint();
		
        	point.position = tempPoint[0];
        	point.velocity = tempPoint[1];
            point.timeDur = TrajectoryPoint.TrajectoryDuration.Trajectory_Duration_10ms;
        	point.profileSlotSelect0 = 0;
		

        	point.zeroPos = false;
	
        	if (pointsCount == 0) {
        		point.zeroPos = true;
        	}
	
        	point.isLastPoint = false;
	
        	if (trajPointsLeft.getIsLastPoint()) {
        		point.isLastPoint = true;
        	}

        	theRobot.leftTrans.setTrajectoryPoint(point);
//    		System.out.println("Left Position " +pointsCount+ ": " + tempPoint[0]);
//    		System.out.println("Left Velocity " +pointsCount+ ": " + tempPoint[1]);

		
        	tempPoint = trajPointsRight.getNextTrajPoint();
				
        	point.position = tempPoint[0];
			point.velocity = tempPoint[1];
			point.profileSlotSelect0 = 0;
	
			point.zeroPos = false;
	
			if (pointsCount == 0) {
				point.zeroPos = true;
			}
	
			point.isLastPoint = false;
	
			if (trajPointsRight.getIsLastPoint()) {
				point.isLastPoint = true;
			}

			theRobot.rightTrans.setTrajectoryPoint(point);
		

//		System.out.println("Right Position " +pointsCount+ ": " + tempPoint[0]);
//		System.out.println("Right Velocity " +pointsCount+ ": " + tempPoint[1]);
		

		
			pointsCount++;
			pointsCountRight = pointsCount;
			pointsCountLeft = pointsCount;
		
			System.out.println("PointsCount: " + pointsCount);
		
        }
	
//	System.out.println("Out of startLoadingPoints");
	
     while((theRobot.leftTrans.getBtmBufferCount()) < 100 || (theRobot.rightTrans.getBtmBufferCount() < 100))
     {
			try {Thread.sleep(10);}
			catch (Exception e ) {	
			}
     }
	
	
	// start MP execution
        theRobot.leftTrans.setMotorModeMotionProfile(SetValueMotionProfile.Enable);
        theRobot.rightTrans.setMotorModeMotionProfile(SetValueMotionProfile.Enable);
	}
	
	public void continueLoadingPoints() {
		
		double [] tempPoint = new double[3];
		
//		theRobot.leftTrans.checkMotionProfileStatus();
//		theRobot.rightTrans.checkMotionProfileStatus();
//		theRobot.leftTrans.displayMotionProfileStatus();
		
		int loadpoints = 0;

		
		while((theRobot.leftTrans.getTopBufferCount() < Constants.kMaxTopBuffer) && (pointsCountLeft < trajPointsLeft.getTotalPoints())
				&& (loadpoints++ < 20))
		{
			tempPoint = trajPointsLeft.getNextTrajPoint();
			
			point.position = tempPoint[0];
			point.velocity = tempPoint[1];
			point.profileSlotSelect0 = 0;
		
			point.zeroPos = false;
		
			if (pointsCount == 0) {
				point.zeroPos = true;
			}
		
			point.isLastPoint = false;
		
			if (trajPointsLeft.getIsLastPoint()) {
				point.isLastPoint = true;
				System.out.println("Last Point for Left");
			}
	
			theRobot.leftTrans.setTrajectoryPoint(point);
			
			pointsCountLeft++;
		}
		
		loadpoints = 0;
		
		while((theRobot.rightTrans.getTopBufferCount() < Constants.kMaxTopBuffer) && (pointsCountRight < trajPointsRight.getTotalPoints())
				&& (loadpoints++ < 20))
		{			
			tempPoint = trajPointsRight.getNextTrajPoint();
			
			point.position = tempPoint[0];
			point.velocity = tempPoint[1];
			point.profileSlotSelect0 = 0;
			
			point.zeroPos = false;
		
			if (pointsCount == 0) {
				point.zeroPos = true;
			}
		
			point.isLastPoint = false;
		
			if (trajPointsRight.getIsLastPoint()) {
				point.isLastPoint = true;
				System.out.println("Last Point for Right");
			}
	
			theRobot.rightTrans.setTrajectoryPoint(point);

			pointsCountRight++;
						
//			System.out.println("PointsCount: " + pointsCount);
		}
		
	}
	
	

	@Override
	public void run() {
		
		boolean interrupted = false;
		
		startLoadingPoints ();
		
		theRobot.finish = System.currentTimeMillis();
		
		System.out.println("Delay = " + (theRobot.finish - theRobot.start));
		
		while(!Thread.currentThread().isInterrupted() && !(theRobot.rightTrans.checkMotionProfileStatus() 
				&& theRobot.leftTrans.checkMotionProfileStatus())) {
			continueLoadingPoints ();
			printToConsole("In Auton Loop.  Interrupted = " + Thread.currentThread().isInterrupted());
			try {Thread.sleep(20);}
			catch (InterruptedException e ) {
				interrupted = true;
			}
		}
		
		//theRobot.ejector.setSpeed(1.0);
		
		if(!interrupted)
		{
			try {
				Thread.sleep(1000);
				//theRobot.ejector.stop();
				//theRobot.ejector.stop();
				//theRobot.elevator.setPosition(0);
			} catch (InterruptedException e) {			
				interrupted = true;
			}
		}
		
	}
	

	@Override
	public String getName() {
		
		return "Switch" ;
	}
	
	int printLoop = 0;
	
	private void printToConsole(String msg)
	{
		if(printLoop++ > 50)
		{
			System.out.println(msg);
			printLoop = 0;
		}
	}
	
}
