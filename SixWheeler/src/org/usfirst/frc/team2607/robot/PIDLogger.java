package org.usfirst.frc.team2607.robot;

import java.io.File;
import java.io.PrintWriter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class PIDLogger extends Thread {
	private PrintWriter logFile = null, logFile2 = null;
	private boolean loggingEnabled = false;
	String deviceName;
	private double SP;
	private long curTime, startTime;
	File src;
	
	TalonSRX srx;
	
	@Override
	public void run(){
		startTime = System.currentTimeMillis();
		while (true){
			logEntry();
			try {Thread.sleep(10); } catch (Exception e) {}
		}
		
	}
	
	public PIDLogger(TalonSRX talonSRX , String dn){
		srx = talonSRX;
		deviceName = dn;
	}
	
	public void updSetpoint(double newSP) {
		SP = newSP;
	}
	
	public void enableLogging(boolean enable) {
	    	boolean okToEnable = enable;
			if (enable && !loggingEnabled) {
	    		if (logFile != null) {
	    			logFile.close();
	    			logFile = null;
	    		}
	    		try {
	    			logFile = new PrintWriter(new File("/home/lvuser/PIDLogger/" + deviceName + System.currentTimeMillis() + ".csv"));
	    			logFile2 = new PrintWriter(new File("/home/lvuser/PIDLogger/" + deviceName + ".csv"));
	    			String header = "Time,TotalTime,SP,NativeVel,Err,Vol,Curr,NativePos,TargetVel";
	    			logFile.println(header);
	    			logFile2.println(header);
	    		} catch (Exception e) {
	    			okToEnable = false;
	    		}
	    	} 
	    	
	    	if (!enable && loggingEnabled) {
	    		if (logFile != null) {
	    			logFile.close();
	    			logFile2.close();
	    			logFile = null;
	    			logFile2 = null;
	    		}
	    	}
	    	
	    	loggingEnabled = okToEnable;
	    }
	 
	 public void logEntry() {
	        if (loggingEnabled) {
	        	curTime = System.currentTimeMillis() - startTime;
	        	String line = System.currentTimeMillis() + "," +
	        					curTime + "," +
	        					SP + "," +
	        					srx.getSelectedSensorVelocity(0) + "," +
	        				    srx.getClosedLoopError(0) + "," + 
	        					srx.getBusVoltage()+ "," + 
	        				    srx.getOutputCurrent() + "," +
	        				    srx.getSelectedSensorPosition(0) + "," +
	        				    ((srx.getControlMode() == ControlMode.PercentOutput) ? "NA" : srx.getClosedLoopTarget(0));
	        	logFile.println(line);
	        	logFile2.println(line);
	        	logFile.flush();
	        	logFile2.flush();
	        }
	    }

}