/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2607.robot;


import org.usfirst.frc.team2607.robot.auto.AutonomousEngine;
//import org.usfirst.frc.team2607.robot.PIDLogger;

import com.ctre.phoenix.motion.MotionProfileStatus;
import com.ctre.phoenix.motion.SetValueMotionProfile;
import com.ctre.phoenix.motion.TrajectoryPoint;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode;
import edu.wpi.cscore.VideoMode.PixelFormat;
import edu.wpi.first.wpilibj.CameraServer;

/*
 * The RobotDrive class has been deprecated and replaced with a number of different
 * classes for each drivetrain.  This was an attempt to use the new library.
 */
// import edu.wpi.first.wpilibj.RobotDrive;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends TimedRobot {

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	public Transmission leftTrans , rightTrans ;
	public RobovikingStick driveController ;
	public DifferentialDrive robotDDrive;
	public ReadTrajectoryPoints trajPointsLeft, trajPointsRight;
	public TrajectoryPoint point = new TrajectoryPoint();
	public PIDLogger rightLogger, leftLogger;
	
	private SerialPort jevois = null;
	private int loopCount;
	private UsbCamera jevoisCam;
	
	private AutonomousEngine autoEngine;
	private Thread autoThread;
	private boolean autonModeRan = false;
	private boolean zombie = false;
	private boolean mpeRunning = false;
	
	public String gameData;
	
	public int count = 0;
	
	public Notifier periodicProcess = new Notifier(new Runnable() {
		@Override
		public void run() {
			leftTrans.processMotionProfileBuffer();
			rightTrans.processMotionProfileBuffer();
		}
	});
	
	public long start = 0;
	public long finish = 0;
	
	@Override
	public void robotInit() {
		
		/*
		 *  Create engine for Autonomous
		 */
		autoEngine=new AutonomousEngine(this);
		autoEngine.loadSavedMode();
		
		/*
		 * In this section we setup the drive train of the robot and get it all ready to be controlled
		 * by the arcadedrive method of the differentialdrive class.
		 */
		
		leftTrans = new Transmission(Constants.leftMotorA , Constants.leftMotorB , "Left");
		rightTrans = new Transmission(Constants.rightMotorA , Constants.rightMotorB , "Right");
		leftTrans.setMotionProfileHighGearGains();
		rightTrans.setMotionProfileHighGearGains();
		rightTrans.setInverted(true);
		leftTrans.setInverted(true);
		leftTrans.setEncoderPhase(true);
		rightTrans.zeroEncoder();
		leftTrans.zeroEncoder();
		driveController = new RobovikingStick(Constants.driverController);
		robotDDrive = new DifferentialDrive(leftTrans, rightTrans);
		robotDDrive.setSafetyEnabled(false);
		
		
		/*
		 * Start the TalonSRX motion profile engine by calling the method processMotionProfileBuffer
		 * every 5 milliseconds.  This does not start the processing of trajectory points.  But it starts
		 * the buffering process to move trajectory points to the bottom buffer.  The MPE has three states:
		 * Disabled, Enabled, and Hold.  The processing of trajectory points starts when the state is changed
		 * to Enabled.
		 */
		
//		periodicProcess.startPeriodic(.005);
		
		/*
		 * This section sets up the Jevois camera.  It sets up a serial port via USB and it also sets up a
		 * camera server to get the video from the Jevois and send it to the Smartdashboard.
		 * 
		 */
		
		gameData = DriverStation.getInstance().getGameSpecificMessage();
		
		/*
		int tryCount = 0;
		do {
			try {
				System.out.print("Trying to create jevois SerialPort...");
				jevois = new SerialPort(9600, SerialPort.Port.kUSB);
				tryCount = 99;
				System.out.println("success!");
			} catch (Exception e) {
				tryCount += 1;
				System.out.println("failed!");
			}
		} while (tryCount < 3);
		
		System.out.println("Starting CameraServer");
		
		if (jevoisCam == null) {
			jevoisCam = CameraServer.getInstance().startAutomaticCapture();
			jevoisCam.setVideoMode(PixelFormat.kYUYV,320,254,60);
			VideoMode vm = jevoisCam.getVideoMode();
			System.out.println("jevoisCam pixel: " + vm.pixelFormat);
			System.out.println("jevoisCam res: " + vm.width + "x" + vm.height);
			System.out.println("jevoisCam fps: " + vm.fps);
		}
		
		if (tryCount == 99) {
			writeJeVois("info\n");
		}
		loopCount = 0;
		
//		jevoisCam = CameraServer.getInstance().startAutomaticCapture(); moved farther up, inside if statement
//		jevoisCam.setBrightness(50);
//		jevoisCam.setExposureManual(50);
//		jevoisCam.setFPS(60);

	}
	
	public void checkJeVois() {
		if (jevois == null) return;
		if (jevois.getBytesReceived() > 0) {
			System.out.println("Waited: " + loopCount + " loops, Rcv'd: " + jevois.readString());
			loopCount = 0;
		}
		if (++loopCount % 150 == 0) {
			System.out.println("checkJeVois() waiting..." + loopCount);
			jevoisCam.setVideoMode(PixelFormat.kYUYV,320,254,60);
			writeJeVois("getpar serout\n");
			writeJeVois("streamoff\n");
			writeJeVois("info\n");
		}
	}
		
	public void writeJeVois(String cmd) {
		if (jevois == null) return;
		int bytes = jevois.writeString(cmd);
		System.out.println("wrote " +  bytes + "/" + cmd.length() + " bytes");	
		loopCount = 0;
	}
	*/
		
		rightLogger = new PIDLogger(leftTrans.getMasterSRX(), "RightTalonSRX");
		leftLogger = new PIDLogger(rightTrans.getMasterSRX(), "LeftTalonSRX");
//		rightLogger.start();
//		leftLogger.start();
// And I am pushing more changes now.
		
	}

	@Override
	public void autonomousInit()
	{
		rightLogger.enableLogging(true);
		leftLogger.enableLogging(true);		
		autoThread = new Thread(autoEngine);
		autoThread.start();
		start = System.currentTimeMillis();
		autonModeRan = true;
		zombie = true;
		mpeRunning = true;
		
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() 
	{
//		leftTrans.checkMotionProfileStatus();
//		rightTrans.checkMotionProfileStatus();
		
		if(count++>5) {
			leftTrans.displayMotionProfileStatus();
//			rightTrans.displayMotionProfileStatus();
			
//			System.out.println("Left Velocity: " + leftTrans.getEncoderVelocity() + 
//					" Right Velocity: " + rightTrans.getEncoderVelocity() + " Right Position: " + rightTrans.getEncoderPosition()
//					+ " Left Position: " + leftTrans.getEncoderPosition());
			count =0;
		}
	}

		/*
		* This function is called during Teleop Initialization
		 */
	@Override	
	public void teleopInit() {
		leftTrans.setLowGearGains();
		rightTrans.setLowGearGains();
		leftTrans.setMotorModePercentOutput();
		rightTrans.setMotorModePercentOutput();

}
	/**
	 * This function is called periodically during operator control.
	 */
	
	boolean isRotating = false;
	
	@Override
	public void teleopPeriodic() {
		
		// checkJeVois();
		
		robotDDrive.arcadeDrive(driveController.getRawAxisWithDeadzone(RobovikingStick.xBoxLeftStickY) , 
				driveController.getRawAxisWithDeadzone(RobovikingStick.xBoxRightStickX));
		
	//	leftTrans.set(0.5);
	//	rightTrans.set(0.5);
	/*	
		if(!isRotating)
		{
			rotateRobot(-90);
			isRotating = true;
		}
		
	*/
		
		if(count>50) {
			System.out.println("Left Velocity: " + leftTrans.getEncoderVelocity() + 
					" Right Velocity: " + rightTrans.getEncoderVelocity() + " Right Position: " + rightTrans.getEncoderPosition()
					+ " Left Position: " + leftTrans.getEncoderPosition());
			count =0;
		} else
		{
			count++;
		}
	}
	
	

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}
	
	
	@Override
	public void disabledPeriodic()
	{
		
			leftTrans.set(0.0);
			rightTrans.set(0.0);
			
			if (autonModeRan)
			{
				if (zombie)
				{
					if(autoThread.isAlive())
					{
						System.out.println("autoThread alive, interrupting");
						autoThread.interrupt();
					}
					else 
					{
						System.out.println("autoThread not alive");
						zombie = false;
					}
				}
				
				if(mpeRunning)
				{
					// Stop MP execution
					leftTrans.setMotorModeMotionProfile(SetValueMotionProfile.Disable);
					rightTrans.setMotorModeMotionProfile(SetValueMotionProfile.Disable);
					periodicProcess.stop();
					System.out.println("Periodic Process stopped.");
					System.out.println("Number of Threads: " + Thread.activeCount());
					rightLogger.enableLogging(false);
					leftLogger.enableLogging(false);		
					mpeRunning = false;
				}
			}
				
			if (driveController.getButtonPressedOneShot(RobovikingStick.xBoxButtonStart)) {
				autoEngine.selectMode();
			}
			
			/*if (driveController.getToggleButton(RobovikingStick.xBoxButtonBack)) {
				climber.setClimber(8);
				SmartDashboard.putString("Climber" , "Right");
			}
			else {
				climber.setClimber(7);
				SmartDashboard.putString("Climber" , "Left");
			}*/

		// checkJeVois();
		
	}
	
	public void rotateRobot(double degrees)
	{
		double deg = degrees;
		double rightTicks = 0;
		double leftTicks = 0;
		
		leftTrans.zeroEncoder();
		rightTrans.zeroEncoder();
		
		if ((deg < 0.0) && (deg > -180.0))
		{
			rightTicks = Constants.ticksPerDegree * deg;
			leftTicks = -Constants.ticksPerDegree * deg;
		}
		else if ((deg > 0.0) && (deg < 180.0))
		{
			rightTicks = Constants.ticksPerDegree * deg;
			leftTicks = -Constants.ticksPerDegree * deg;
		}
		else
		{
			rightTicks = 0;
			leftTicks = 0;
		}
		
		System.out.println("Left Ticks = " + leftTicks + " Right Ticks = " + rightTicks);
		System.out.println("Right Position: " + rightTrans.getEncoderPosition()
				+ " Left Position: " + leftTrans.getEncoderPosition());
		
		leftTrans.setPositionModeGains();
		rightTrans.setPositionModeGains();
		
		leftTrans.setPosition(leftTicks);
		rightTrans.setPosition(rightTicks);
	}
}
