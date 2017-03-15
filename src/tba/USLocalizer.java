/*  Thomas Philippon and Olisa Okonkwo group 30 
 * this class localize the heading of the robot using two different methods
 * In the demo we used falling edge because it was working better
 * during the tests
 */

package tba;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

import lejos.robotics.SampleProvider;

import java.io.FileWriter;
import java.io.PrintWriter;
import java.io.IOException;

/**
 * The USLocalization class uses ultra-sonic sensors to locate the walls surrounding 
 * the robot, where this information is used to locate the 0 degree angle.*/

public class USLocalizer {
	public enum LocalizationType { FALLING_EDGE, RISING_EDGE };
	public static double ROTATION_SPEED =150;
	public static double W_radi= 2.2;
	public static double  W_base=10.0;
	private Odometer odo;
	private Navigation nav;
	private SampleProvider usSensor;
	private float[] usData;
	private LocalizationType locType;
	//Gap constant to dead with the sensor noise
	private double Gap = 4;
	//constant of the wall distance 
	private double Wall_dist=35;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	private float[] medianFilter = new float[]{0,0,0};
	
	
	
	public USLocalizer(Odometer odo, Navigation nav, SampleProvider usSensor, float[] usData, LocalizationType locType) {
		this.odo = odo;
		this.nav=nav;
		this.usSensor = usSensor;
		this.usData = usData;
		this.locType = locType;
		EV3LargeRegulatedMotor[] motors = this.odo.getMotors();
		this.leftMotor = motors[0];
		this.rightMotor = motors[1];
	}
	
	public void doLocalization() {
		//double [] pos = new double [3];
		double angleA, angleB;
			leftMotor.setSpeed(50);
			rightMotor.setSpeed(50);
	
			if (locType == LocalizationType.FALLING_EDGE) {
				/*
				 * The robot should turn until it sees no wall, then look for the
				 * "falling edges:" the points where it sees the wall.
				 * will face the opposite of the wall for most of it.
				 */
				
				
			// rotate the robot until it sees no wall
			while ( getFilteredData() < (Wall_dist + Gap)){
			
				leftMotor.forward();
				rightMotor.backward();
			}
			
			// keep rotating until the robot sees a wall, then latch the angle
			while ( getFilteredData() > Wall_dist){
				leftMotor.forward();
				rightMotor.backward();
			}
			//right wall detected
	     	angleA= odo.getAng();
			
			// switch direction and wait until it sees no wall
				while (getFilteredData() < (Wall_dist+Gap)){
	
					leftMotor.backward();
					rightMotor.forward();
				}
				
			// keep rotating until the robot sees a wall, then latch the angle
				while(getFilteredData() > Wall_dist){
					leftMotor.backward();
					rightMotor.forward();
				}
				
				//stop motors
				rightMotor.stop(true);
				leftMotor.stop(true);
				
				//left wall detected
				angleB = odo.getAng();
			
		       // make sure the angle A is always smaller than the angle B
				if(angleA > angleB){
					angleA = angleA - 360;
				}
			
				//compute the average angle between A and B
				double averageAngle = (angleA + angleB)/2;
				
				//compute the angle where the robot will be facing North, we add 228 to make to robot towards x axis, zero degrees 
				double ninety_D =  angleB - averageAngle +230 ;  // had to modify it to make it more accurate 
			
				//Robot rotates to North
				leftMotor.rotate((int) ninety_D, true);
				rightMotor.rotate((int)- ninety_D, false);
			
				odo.setPosition(new double [] {0.0, 0.0, 0.0}, new boolean [] {true, true, true});
			
		} 
		
		else {
			/*
			 * The robot should turn until it sees the wall, then look for the
			 * "rising edges:" the points where it no longer sees the wall.
			 * This is very similar to the FALLING_EDGE routine, but the robot
			 * will face toward the wall for most of it.
			 */
			
			// rotate the robot until it sees wall
			while (getFilteredData() >(Wall_dist-Gap)){
				leftMotor.backward();
				rightMotor.forward();
			}
			
			//If the robot is facing the wall turn until it sees no wall
			while (getFilteredData()< Wall_dist){
				leftMotor.backward();
				rightMotor.forward();
			}
			//right wall
			angleA= odo.getAng();
			

			// switch direction and wait until it sees no wall
				while (getFilteredData() > (Wall_dist-Gap)){
	
					leftMotor.forward();
					rightMotor.backward();
				}
				
			// keep rotating until the robot sees no wall, then latch the angle
			while(getFilteredData() < Wall_dist){
				leftMotor.forward();
				rightMotor.backward();
			}
			
			//stop motor 
			rightMotor.stop(true);
			leftMotor.stop(true);
			
			//left wall
			angleB = odo.getAng();		
			
			// make sure the angle A is always smaller than the angle B
			if(angleA > angleB){
				angleA = angleA - 360;
			}
		
			//compute the average angle between A and B
			double averageAngle = (angleA + angleB)/2;
			
			//compute the angle where the robot will be facing North
			double ninety_D =  angleB - averageAngle + 230;
		
			//Robot rotates to North
			leftMotor.rotate((int) ninety_D, true);
			rightMotor.rotate((int)- ninety_D, false);
		
			odo.setPosition(new double [] {0.0, 0.0, 0.0}, new boolean [] {true, true, true});
		
		}
	}
	
	private float getFilteredData() {
       //retrieve data from the sensor 
		usSensor.fetchSample(usData, 0);
		float distance = (usData[0]*100);
		try {
			writeToFile(String.valueOf(distance), "SensorData.txt");
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		//Filter to set garbage values as being 200 cm 
		if (distance >200){
			distance =200;
		}
		medianFilter[0]=medianFilter[1];
		medianFilter[1]=medianFilter[2];
		medianFilter[2]=distance;
		
		if(medianFilter[0]<medianFilter[1]){
			if(medianFilter[0]>medianFilter[2]){
				distance = medianFilter[0];
			}
			else if(medianFilter[1]<medianFilter[2]){
				distance = medianFilter[1];
			}
			else{
				distance = medianFilter[2];
			}
		}
		else{
			if(medianFilter[1]>medianFilter[2]){
				distance = medianFilter[1];
			}
			else if(medianFilter[0]<medianFilter[2]){
				distance = medianFilter[0];
			}
			else{
				distance = medianFilter[2];
			}
		}
		
		return distance;
	}
	
	public void writeToFile(String textLine, String path) throws IOException {
		
		boolean append_to_file = true;
		FileWriter write = new FileWriter( path , append_to_file);
		PrintWriter print_line = new PrintWriter( write );
		print_line.printf( "%s" + "%n" , textLine);
		print_line.close();
		
	}
	
}
