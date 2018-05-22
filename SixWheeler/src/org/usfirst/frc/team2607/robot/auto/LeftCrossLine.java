package org.usfirst.frc.team2607.robot.auto;

import org.usfirst.frc.team2607.robot.Constants;
import org.usfirst.frc.team2607.robot.MotionProfile;
import org.usfirst.frc.team2607.robot.ReadTrajectoryPoints;
import org.usfirst.frc.team2607.robot.Robot;

import com.ctre.phoenix.motion.SetValueMotionProfile;
import com.ctre.phoenix.motion.TrajectoryPoint;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;

public class LeftCrossLine extends AutonomousMode {
	
	Robot theRobot ;
	public ReadTrajectoryPoints trajPointsLeft, trajPointsRight;
	public TrajectoryPoint point = new TrajectoryPoint();
	private int pointsCount = 0;
	private int pointsCountRight = 0;
	private int pointsCountLeft = 0;
	
	public LeftCrossLine (Robot r) {
		theRobot = r;
	}
	
	private Notifier periodicProcess = new Notifier(new Runnable() {
		@Override
		public void run() {
			theRobot.leftTrans.processMotionProfileBuffer();
			theRobot.rightTrans.processMotionProfileBuffer();
		}
	});
	
	public void autonomousInit () {
		
//		theRobot.elevator.setPosition(2000);
		
		double [] tempPoint = new double [3];
		
		theRobot.leftTrans.setInverted(true);
		
		theRobot.leftTrans.setHighGear(false, true);
		theRobot.rightTrans.setHighGear(false, true);
//		theRobot.shifter.set(false);
	
		theRobot.leftTrans.setClearTrajectoryBuffer();
		theRobot.rightTrans.setClearTrajectoryBuffer();
	
		
      
            
        trajPointsRight = new ReadTrajectoryPoints();
        trajPointsRight.addTrajectoryPoints("rw_traj_LeftCrossLine_ticks.csv",1.0);
        trajPointsLeft = new ReadTrajectoryPoints();
        trajPointsLeft.addTrajectoryPoints("lw_traj_LeftCrossLine_ticks.csv",1.0);
             
        
    
        point.timeDur = TrajectoryPoint.TrajectoryDuration.Trajectory_Duration_10ms;
	

	
        pointsCount = 0;
	
	/*
	 * The loop before loads trajectory points to the top buffer of both transmissions, the right and the left side
	 * transmissions.  It will load points to the top buffer until it either loads all the points in the trajectory
	 * or until it fills the top buffer.  The maximum size of the top buffer (in trajectory points) is defined in
	 * Constant.kMaxTopBuffer.
	 */
	
        while ((pointsCount < trajPointsLeft.getTotalPoints()) && (theRobot.leftTrans.getTopBufferCount() < Constants.kMaxTopBuffer)) {
		
        	tempPoint = trajPointsLeft.getNextTrajPoint();
		
        	point.position = tempPoint[0];
        	point.velocity = tempPoint[1];
        	point.profileSlotSelect0 = 0;
		
		
		//point.position = MotionProfile.PointsLeft[pointsCount][0];
		//point.velocity = MotionProfile.PointsLeft[pointsCount][1];
		//point.profileSlotSelect0 = 0;
		
	
        	point.zeroPos = false;
	
        	if (pointsCount == 0) {
        		point.zeroPos = true;
        	}
	
        	point.isLastPoint = false;
	
        	if (trajPointsLeft.getIsLastPoint()) {
        		point.isLastPoint = true;
        	}

        	theRobot.leftTrans.setTrajectoryPoint(point);
		
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
/*		
		System.out.println("Left Position " +pointsCount+ ": " + MotionProfile.PointsLeft[pointsCount][0]);
		System.out.println("Left Velocity " +pointsCount+ ": " + MotionProfile.PointsLeft[pointsCount][1]);
		System.out.println("Left Interval " +pointsCount+ ": " + MotionProfile.PointsLeft[pointsCount][2]);
		System.out.println("Right Position " +pointsCount+ ": " + MotionProfile.PointsRight[pointsCount][0]);
		System.out.println("Right Velocity " +pointsCount+ ": " + MotionProfile.PointsRight[pointsCount][1]);
		System.out.println("Right Interval " +pointsCount+ ": " + MotionProfile.PointsRight[pointsCount][2]);
*/
		
		pointsCount++;
		pointsCountRight = pointsCount;
		pointsCountLeft = pointsCount;
		
		//	System.out.println("PointsCount: " + pointsCount);
		
     //  }
	
//	System.out.println("Out of AutonomousInit");
	
	
	
	// start MP execution
        periodicProcess.startPeriodic(.005);
        theRobot.leftTrans.setMotorModeMotionProfile(SetValueMotionProfile.Enable);
        theRobot.rightTrans.setMotorModeMotionProfile(SetValueMotionProfile.Enable);
        }
	}
	public void autonomousPeriodic() {
		
		double [] tempPoint = new double[3];
		
//		theRobot.leftTrans.checkMotionProfileStatus();
//		theRobot.rightTrans.checkMotionProfileStatus();
		theRobot.leftTrans.displayMotionProfileStatus();

		
		if((theRobot.leftTrans.getTopBufferCount() < Constants.kMaxTopBuffer) && (pointsCountLeft < trajPointsLeft.getTotalPoints()))
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
			}
	
			theRobot.leftTrans.setTrajectoryPoint(point);
			
			pointsCountLeft++;
		}
		
		if((theRobot.rightTrans.getTopBufferCount() < Constants.kMaxTopBuffer) && (pointsCountRight < trajPointsRight.getTotalPoints()))
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
			}
	
			theRobot.rightTrans.setTrajectoryPoint(point);

			pointsCountRight++;
						
//			System.out.println("PointsCount: " + pointsCount);
		}
		
	}

	@Override
	public void run() {
		autonomousInit ();
		
		//theRobot.elevator.setPosition(15000);
		
		while((!theRobot.rightTrans.checkMotionProfileStatus()) && !theRobot.leftTrans.checkMotionProfileStatus()) {
			autonomousPeriodic ();
			try {Thread.sleep(20);}
			catch (Exception e ) {	
			}
		}
		
//		theRobot.elevator.setPosition(0);

		
	}
	

	@Override
	public String getName() 
		{
		return "LeftCrossLine" ;
		}
	}
