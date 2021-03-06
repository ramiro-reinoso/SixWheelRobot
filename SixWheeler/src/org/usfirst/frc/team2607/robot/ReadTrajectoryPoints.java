package org.usfirst.frc.team2607.robot;

/*
 * This class loads a trajectory file into memory.  The trajectory file is a comma delimited file.
 * Each row has three members separated by a comma:
 * 
 * <distance>,<velocity>,<interval>
 * 
 * where distance is in native encoder ticks, velocity is in native encoder ticks per 100 milliseconds, and interval
 * is in milliseconds.
 * 
 * The trajectory file is generated by a software tool "PathGenerator" which is based on Pathfinder.
 * 
 */

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;

public class ReadTrajectoryPoints {

	double [][] trajectoryPoints = new double [5000][3];	// Two dimensional array of double numbers that hold the trajectory points.
															// The maximum number of trajectory points is 5000

	boolean lastPoint = false;	
	int pointPointer = 0;
	int totalPoints = 0;
	double lastDistancePoint = 0.0;
    int bufferPointer = 0;
	
	public boolean addTrajectoryPoints(String filename, double fudge)
	{
        String trajFile = "/home/lvuser/trajectories/" + filename;	// This points to the RoboRio repository
        String line = "";
        String splitChar = ",";						// Comma separated file.
        String [] trajPoint = new String[3];
        
        double fudgeMult = fudge;
        
        try (BufferedReader br = new BufferedReader(new FileReader(trajFile))) {

            while ((line = br.readLine()) != null) {
            	
                trajPoint = line.split(splitChar);
                
//                System.out.println("Pointer " + bufferPointer + " traj1 " + trajPoint[0] + " traj2 " + trajPoint[1] + " traj3 " + trajPoint[2]);

                trajectoryPoints[bufferPointer][0]= fudgeMult * Double.parseDouble(trajPoint[0])  + lastDistancePoint;
                trajectoryPoints[bufferPointer][1]= fudgeMult * Double.parseDouble(trajPoint[1]);
                trajectoryPoints[bufferPointer][2]= Double.parseDouble(trajPoint[2]);              
               
//               System.out.println(trajectoryPoints[bufferPointer][0] + " " + trajectoryPoints[bufferPointer][1] + 
//               		" " + trajectoryPoints[bufferPointer][2] + " Count= " + bufferPointer);

                bufferPointer++;                                
            }
            
            br.close();
            
            
            lastDistancePoint = trajectoryPoints[bufferPointer-1][0];
            totalPoints = bufferPointer;

//            System.out.println("totalPoints = " + totalPoints);

        } catch (IOException e) {
            e.printStackTrace();
        }
        
        return true;

	}
	
	public double [] getNextTrajPoint( ) {
		return trajectoryPoints[pointPointer++];	
	}
	
	public int getTotalPoints() {
		return totalPoints;
	}
	
	public boolean getIsLastPoint() {
		return (pointPointer == totalPoints);
	}
	
	public double [] getTrajPoint(int index) {
		return trajectoryPoints[index];
	}
}
