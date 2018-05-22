package org.usfirst.frc.team2607.robot;

import com.ctre.phoenix.motion.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.*;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.SpeedController;

public class Transmission implements SpeedController{
	
	private TalonSRX motor1;
	private TalonSRX motor2 = null;
	private String name;
	
	//PIDLogger logger;
	
	// switch this to initialize to false if the transmissions default to low gear
	private boolean highGear = true;
	
	// keep track of whether or not the Transmission is inverted via calls to setInverted
	private boolean inverted = false;
	
	// default to running in % output mode
	private ControlMode motorMode = ControlMode.PercentOutput;
	private MotionProfileStatus motionProfileStatus = new MotionProfileStatus();
	private MotionProfileStatus _status = new MotionProfileStatus();

	public static final double maxSpeedLowGearRight = 200.0; // Maximum speed in ticks per 100 msec
	public static final double maxSpeedLowGearLeft = 200.0;  // Maximum speed in ticks per 100 msec
	public static final double maxSpeedHighGearRight = 400.0;  // Maximum speed in ticks per 100 msec
	public static final double maxSpeedHighGearLeft = 400.0;   // Maximum speed in ticks per 100 msec
	
	
	public Transmission(int channelA , int channelB , String name){
		motor1 = new TalonSRX(channelA);
		motor2 = new TalonSRX(channelB);
		this.name = name;
		
		motor2.follow(motor1);
		motor2.setNeutralMode(NeutralMode.Brake);
		
		motor1.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
		
		// set "sensor phase" true if you need to reverse the encoder, i.e. have it read positive
		// values when you apply positive voltage to the motor.  Encoder and motor must be "in phase",
		// encoder reading positive/increasing with positive motor voltage, and reading negative/decreasing
		// with negative motor voltage
		motor1.setSensorPhase(false);			
		
		// nominal and peak output is %, i.e 1.0 is full forward, -1.0 full reverse
		motor1.configNominalOutputForward(0.0,0);
		motor1.configNominalOutputReverse(0.0, 0);
		motor1.configPeakOutputForward(1.0, 0);
		motor1.configPeakOutputReverse(-1.0, 0);
		
		motor2.configNominalOutputForward(0.0,0);
		motor2.configNominalOutputReverse(0.0, 0);
		motor2.configPeakOutputForward(1.0, 0);
		motor2.configPeakOutputReverse(-1.0, 0);
		
		motor1.configOpenloopRamp(0.30, 0);
		motor2.configOpenloopRamp(0.30, 0);
		motor1.configClosedloopRamp(0.25, 0);
		motor2.configClosedloopRamp(0.25, 0);
		
		motor1.setNeutralMode(NeutralMode.Brake);

		motor1.changeMotionControlFramePeriod(5);
	 
		// set the initial gains to high gear by default
		// if low gear is the default, update this to call setLowGearGains() instead
		setHighGearGains();
		//setLowGearGains();		
						
		//logger = new PIDLogger(motor1, name);
		//logger.start();
	}

	public Transmission(int channelA , String name){
		motor1 = new TalonSRX(channelA);
		this.name = name;
		
		motor1.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
		
		// set "sensor phase" true if you need to reverse the encoder, i.e. have it read positive
		// values when you apply positive voltage to the motor.  Encoder and motor must be "in phase",
		// encoder reading positive/increasing with positive motor voltage, and reading negative/decreasing
		// with negative motor voltage
		motor1.setSensorPhase(false);			
		
		// nominal and peak output is %, i.e 1.0 is full forward, -1.0 full reverse
		motor1.configNominalOutputForward(0.0,0);
		motor1.configNominalOutputReverse(0.0, 0);
		motor1.configPeakOutputForward(1.0, 0);
		motor1.configPeakOutputReverse(-1.0, 0);		
		motor1.configOpenloopRamp(0.30, 0);
		motor1.configClosedloopRamp(0.25, 0);		
		motor1.setNeutralMode(NeutralMode.Brake);
		motor1.changeMotionControlFramePeriod(5);
	 
		// set the initial gains to high gear by default
		// if low gear is the default, update this to call setLowGearGains() instead
		setHighGearGains();
		//setLowGearGains();		
						
		//logger = new PIDLogger(motor1, name);
		//logger.start();
	}
	
	protected void setLowGearGains() {
		/*
		 * PID gains will need to be determined for high gear and low gear if we want to use velocity PID or
		 * position PID.  Follow tuning procedure in SRX documentation
		 */

		if (name.equalsIgnoreCase("Right")) {
			motor1.config_kP(0, 4.5, 0);
			motor1.config_kI(0, 0.0, 0);
			motor1.config_kD(0, 0.0, 0);
			motor1.config_kF(0, 2.48, 0);
		}
		else
		{
			motor1.config_kP(0, 4.5, 0);
			motor1.config_kI(0, 0.0, 0);
			motor1.config_kD(0, 0.0, 0);
			motor1.config_kF(0, 2.41, 0);			
		}
	}
	
	protected void setHighGearGains() {
		/*
		 * PID gains will need to be determined for high gear and low gear if we want to use velocity PID or
		 * position PID.  Follow tuning procedure in SRX documentation
		 */
		if (name.equalsIgnoreCase("Right")) {
			motor1.config_kP(0, 0.3, 0);//0.1
			motor1.config_kI(0, 0.0, 0);
			motor1.config_kD(0, 2.0, 0);
			motor1.config_kF(0, 1.27, 0);
		}
		else
		{
			motor1.config_kP(0, 0.3, 0);//0.1
			motor1.config_kI(0, 0.0, 0);
			motor1.config_kD(0, 2.3, 0);
			motor1.config_kF(0, 1.26, 0);			
		}

	}
	
	protected void setMotionProfileLowGearGains() {
		/*
		 * PID gains will need to be determined for high gear and low gear if we want to use velocity PID or
		 * position PID.  Follow tuning procedure in SRX documentation
		 */
		
		if (name.equalsIgnoreCase("Right")) {
			motor1.config_kP(0, 1.0, 0); //1.0
			motor1.config_kI(0, 0.0, 0);
			motor1.config_kD(0, 0.0, 0); 
			motor1.config_kF(0, 2.48, 0);//2.48
		}
		else
		{
			motor1.config_kP(0, 1.0, 0);//1.0
			motor1.config_kI(0, 0.0, 0);
			motor1.config_kD(0, 0.0, 0);
			motor1.config_kF(0, 2.41, 0);	//2.41		
		}
		
	}

	protected void setMotionProfileHighGearGains() {
		/*
		 * PID gains will need to be determined for high gear and low gear if we want to use velocity PID or
		 * position PID.  Follow tuning procedure in SRX documentation
		 */
		if (name.equalsIgnoreCase("Right")) {
			motor1.config_kP(0, 0.0, 0);
			motor1.config_kI(0, 0.0, 0);
			motor1.config_kD(0, 0.0, 0);
			motor1.config_kF(0, 2.48, 0);
		}
		else
		{
			motor1.config_kP(0, 0.0, 0);
			motor1.config_kI(0, 0.0, 0);
			motor1.config_kD(0, 0.0, 0);
			motor1.config_kF(0, 2.41, 0);			
		}
	}
	
	protected void setPositionLowGearGains() {
		/*
		 * PID gains will need to be determined for high gear and low gear if we want to use velocity PID or
		 * position PID.  Follow tuning procedure in SRX documentation
		 */
		if (name.equalsIgnoreCase("Right")) {
			motor1.config_kP(0, 1.0, 0);
			motor1.config_kI(0, 0.0, 0);
			motor1.config_kD(0, 0.0, 0);
			motor1.config_kF(0, 0.0, 0);
		}
		else
		{
			motor1.config_kP(0, 1.0, 0);
			motor1.config_kI(0, 0.0, 0);
			motor1.config_kD(0, 0.0, 0);
			motor1.config_kF(0, 0.0, 0);			
		}
	}
	
	// set the appropriate PID gains for the gear we're currently in (hg = true for high gear, false for low gear)
	// this doesn't actually shift the solenoid, just sets the PID gains
	public void setHighGear(boolean hg, boolean following) {
		if (highGear != hg) {
			highGear = hg;
			if (highGear) {
				if (following) {
					setMotionProfileHighGearGains();
				} else {
					setHighGearGains();	
					System.out.println("High Gear Gains set." + highGear);
				}
			} else { 
				if (following) {
					setMotionProfileLowGearGains();
				} else {
					setLowGearGains();
					System.out.println("Low Gear Gains set." + highGear);
				}
			}
		}
	}
	
	@Override
	public void pidWrite(double output) {
		
	}
	
	public double getOutputVoltage()
	{
		return motor1.getMotorOutputVoltage();
	}
	
	public double getOutputCurrent()
	{
		return motor1.getOutputCurrent();
	}
	
	@Override
	public double get() {
		return motor1.getMotorOutputPercent();
	}
	
	public void setMotorModePercentOutput() {
		motorMode = ControlMode.PercentOutput;
	}
	
	public void setMotorModeVelocity() {
		motorMode = ControlMode.Velocity;
	}

	// return current distance traveled (encoder ticks) 
	public double getDistanceTicks() {
		return ((double)motor1.getSelectedSensorPosition(0));
	}
	
	// zeroes the encoder position
	public void zeroEncoder() {
		motor1.setSelectedSensorPosition(0, 0, 0);
	}
	
	public double getEncoderVelocity() {
		// Get the encoder velocity in native units per 100 msecs
		
		return ((double)motor1.getSelectedSensorVelocity(0));
	}
	
	public void setEncoderPhase(boolean phase) {
		motor1.setSensorPhase(phase);
	}
	
	// set brake mode on or off
	public void setBrakeMode(boolean brotherbear) {
		motor1.setNeutralMode((brotherbear) ? NeutralMode.Brake : NeutralMode.Coast);
		if(motor2 != null)
		{
			motor2.setNeutralMode((brotherbear) ? NeutralMode.Brake : NeutralMode.Coast);
		}
	}
	
	@Override
	public void set(double speed) {
		if (motorMode == ControlMode.PercentOutput) {
			// input speed param is % output, -1.0 to 1.0
			motor1.set(motorMode, speed);
		} else if (motorMode == ControlMode.Velocity) {
			// innput is in ticks per 100 msec
			if (highGear) 
			{
				if (name.equalsIgnoreCase("Right")) 
				{
					motor1.set(motorMode, speed * maxSpeedHighGearRight);
				}
				else
				{
					motor1.set(motorMode,  speed * maxSpeedHighGearLeft);
				}
			}
			else 
			{
				if (name.equalsIgnoreCase("Right")) 
				{
				motor1.set(motorMode, speed * maxSpeedLowGearRight);
				}
				else 
				{
				motor1.set(motorMode,  speed * maxSpeedLowGearLeft);
				}
			}
		}
		//logger.updSetpoint(speed);
	}
	
	@Override
	public void setInverted(boolean isInverted) {
		inverted = isInverted;
		motor1.setInverted(isInverted);
		if(motor2 != null)
		{
			motor2.setInverted(isInverted);
		}
	}
	
	@Override
	public boolean getInverted() {
		return inverted;
	}
	
	@Override
	public void disable() {
		// TODO Auto-generated method stub
		
	}
	
	@Override
	public void stopMotor() {
		// TODO Auto-generated method stub
		
	}

	public String getName() {
		return name;
	}
	
	public int getTopBufferCount() {
		motor1.getMotionProfileStatus(_status);
		return _status.topBufferCnt;
	}
	
	public int getBtmBufferCount() {
		motor1.getMotionProfileStatus(_status);
		return _status.btmBufferCnt;
	}
	
	public TalonSRX getMasterSRX() {
		return motor1;
	}
	
	public void setTrajectoryPoint (TrajectoryPoint newpoint){
		motor1.pushMotionProfileTrajectory(newpoint);
	}
	
	public void processMotionProfileBuffer () {
		motor1.processMotionProfileBuffer();
	}
	
	public void setMotorModeMotionProfile(SetValueMotionProfile setValue) {
		motor1.set(ControlMode.MotionProfile, setValue.value);
	}
	
	public void setClearTrajectoryBuffer() {
		motor1.clearMotionProfileTrajectories();
	}
	
	public void setClearHasUnderrun () {
		motor1.clearMotionProfileHasUnderrun(0);
	}
	

	public boolean checkMotionProfileStatus() {
		motor1.getMotionProfileStatus(motionProfileStatus);
//		if (motionProfileStatus.outputEnable == SetValueMotionProfile.Enable && motionProfileStatus.isLast) {
		if (motionProfileStatus.isLast) {
			System.out.println("PROCESSED LAST POINT");
				motor1.set(ControlMode.MotionProfile, SetValueMotionProfile.Disable.value);
			return true;
		}
		return false;		
	}
	
	public void displayMotionProfileStatus() {
		motor1.getMotionProfileStatus(_status);
		Instrumentation.process(_status,0.0,0.0,0.0);
	}
	
	public void setInitPositionControlMode()
	{
		setPositionLowGearGains();
		motor1.setSelectedSensorPosition(0, 0, 0);
	}
	
	public void setPosition(double ticks)
	{
		motor1.set(ControlMode.Position,ticks);
		System.out.println("Setting Transmission Position");
	}
	
	protected void setPositionModeGains() {
		/*
		 * PID gains will need to be determined for high gear and low gear if we want to use velocity PID or
		 * position PID.  Follow tuning procedure in SRX documentation
		 */
		if (name.equalsIgnoreCase("Left")) {
			/*
			 * These are the PID gains for the Right transmission
			 */
			motor1.config_kP(0, 1.0, 0);
			motor1.config_kI(0, 0.0, 0);
			motor1.config_kD(0, 0.0, 0);
			motor1.config_kF(0, 0.0, 0);
		} else {
			/*
			 * These are the PID gains for the Left transmission
			 */
			motor1.config_kP(0, 1.0, 0);
			motor1.config_kI(0, 0.0, 0);
			motor1.config_kD(0, 0.0, 0);
			motor1.config_kF(0, 0.0, 0);
		}
	}
	
	public double getEncoderPosition() {
		return motor1.getSelectedSensorPosition(0);
	}
	
	public void setCoast() {
		motor1.setNeutralMode(NeutralMode.Coast);
		if(motor2 != null)
		{
			motor2.setNeutralMode(NeutralMode.Coast);
		}
	}
	
	public void setBrake() {
		motor1.setNeutralMode(NeutralMode.Brake);
		if(motor2 != null)
		{
			motor2.setNeutralMode(NeutralMode.Brake);
		}
	}
	
	
	
}