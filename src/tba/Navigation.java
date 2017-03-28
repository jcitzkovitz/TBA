	package tba;

/*
 * File: Navigation.java

 * Written by: Sean Lawlor
 * ECSE 211 - Design Principles and Methods, Head TA
 * Fall 2011
 * Ported to EV3 by: Francois Ouellet Delorme
 * Fall 2015
 * 
 * Movement control class (turnTo, travelTo, flt, localize)
 */

import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.robotics.SampleProvider;

/**
 * The navigation class controls how the robot will maneuver about the grid.
 * The robot navigates along the x and y axes only, thus at no angles other
 * than 0 degrees, 90 degrees, 180 degrees and 270 degrees.*/

public class Navigation {
	final static int FAST = 150, SLOW = 100, ACCELERATION = 1000, ACCELERATION_SLOW = 1000;
	final static double DEG_ERR = 2.5, CM_ERR = 1.0, TILE_LENGTH = 30.48, rightSensorToBack = 15.8, rightSensorToFront = 11;
	private Odometer odometer;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	private static boolean isTurning = false;
	private boolean rightFirst, correctHeading = false;
	private double correctionAngle = 0;
	SampleProvider usSensorR; 
	float[] usDataR;
	SampleProvider usSensorF; 
	float[] usDataF;
	

	public Navigation(Odometer odo, SampleProvider usSensorR, float[] usDataR, SampleProvider usSensorF, float[] usDataF) {
		this.odometer = odo;
		EV3LargeRegulatedMotor[] motors = this.odometer.getMotors();
		this.leftMotor = motors[0];
		this.rightMotor = motors[1];
		this.usSensorR = usSensorR;
		this.usDataR = usDataR;
		this.usSensorF = usSensorF;
		this.usDataF = usDataF;

		// set acceleration
		this.leftMotor.setAcceleration(ACCELERATION);
		this.rightMotor.setAcceleration(ACCELERATION);
	}

	/*
	 * Functions to set the motor speeds jointly
	 */
	public void setSpeeds(float lSpd, float rSpd) {
		this.leftMotor.setSpeed(lSpd);
		this.rightMotor.setSpeed(rSpd);
		if (lSpd < 0)
			this.leftMotor.backward();
		else
			this.leftMotor.forward();
		if (rSpd < 0)
			this.rightMotor.backward();
		else
			this.rightMotor.forward();
	}

	public void setSpeeds(int lSpd, int rSpd) {
		this.leftMotor.setSpeed(lSpd);
		this.rightMotor.setSpeed(rSpd);
		if (lSpd < 0)
			this.leftMotor.backward();
		else
			this.leftMotor.forward();
		if (rSpd < 0)
			this.rightMotor.backward();
		else
			this.rightMotor.forward();
	}

	/*
	 * Float the two motors jointly
	 */
	public void setFloat() {
		this.leftMotor.stop();
		this.rightMotor.stop();
		this.leftMotor.flt(true);
		this.rightMotor.flt(true);
	}

	/*
	 * TravelTo function which takes as arguments the x and y position in cm Will travel to designated position, while
	 * constantly updating it's heading
	 */
	public void travelTo(double x, double y) {
		this.leftMotor.setAcceleration(ACCELERATION_SLOW);
		this.rightMotor.setAcceleration(ACCELERATION_SLOW);
		boolean posY,posX;

		
		/* The navigation will work as follows: the robot will travel in the y direction followed by the
		 * x direction. This is done in order to more easily correct odometry with the light sensor as 
		 * the robot will only be traveling along the x and y axes*/
		
		// Turn in the direction of the positive y axis (90 degrees)

		if(y - odometer.getY() >= 0)
		{
			this.turnTo(90, true);
			posY=true;
		}
		
		// Turn in the direction of the negative y axis (270 degrees)
		else
		{
			this.turnTo(270, true);
			posY=false;
		}
		
		
		while(Math.abs(y - odometer.getY()) > CM_ERR)
		{
			if(getFilteredDataF() < 20)
			{
				Sound.beep();
				if(posY)
				{
					this.turnTo(180, true);
					if(this.odometer.getX() <= 6*TILE_LENGTH)
					{
						while(getFilteredDataR() < 30)
						{
							this.setSpeeds(-SLOW,-SLOW);
						}
						double currentX = this.odometer.getX();
						while(Math.abs(currentX-this.odometer.getX()) < rightSensorToFront)
						{
							this.setSpeeds(-SLOW,-SLOW);
						}
						this.setSpeeds(0,0);
					}
					else
					{
						while(getFilteredDataR() < 20)
						{
							this.setSpeeds(SLOW,SLOW);
						}
						double currentX = this.odometer.getX();
						while(Math.abs(currentX-this.odometer.getX()) < rightSensorToBack)
						{
							this.setSpeeds(SLOW,SLOW);
						}
						this.setSpeeds(0,0);
					}
				}
				else
				{
					this.turnTo(0, true);
					if(this.odometer.getX() <= 6*TILE_LENGTH)
					{
						while(getFilteredDataR() < 20)
						{
							this.setSpeeds(SLOW,SLOW);
						}
						double currentX = this.odometer.getX();
						while(Math.abs(currentX-this.odometer.getX()) < rightSensorToBack)
						{
							this.setSpeeds(SLOW,SLOW);
						}
						this.setSpeeds(0,0);
					}
					else
					{
						while(getFilteredDataR() < 20)
						{
							this.setSpeeds(-SLOW,-SLOW);
						}
						double currentX = this.odometer.getX();
						while(Math.abs(currentX-this.odometer.getX()) < rightSensorToFront)
						{
							this.setSpeeds(-SLOW,-SLOW);
						}
						this.setSpeeds(0,0);
					}
				}
				
				this.travelTo(x, y);
				return;
			}
			else
			{
				
				if(!correctHeading)
				{
					this.setSpeeds(FAST,FAST);
				}
				else
				{
					this.setSpeeds(0,0);
					try{Thread.sleep(500);}catch(Exception e){}
					this.setSpeeds(SLOW,SLOW);
					if(rightFirst)
					{
						this.leftMotor.rotate(convertAngle(odometer.getWheelRadius(),odometer.getBaseWidth(),correctionAngle),true);
						this.rightMotor.rotate(-convertAngle(odometer.getWheelRadius(),odometer.getBaseWidth(),correctionAngle),false);
					}
					else
					{
						this.leftMotor.rotate(-convertAngle(odometer.getWheelRadius(),odometer.getBaseWidth(),correctionAngle),true);
						this.rightMotor.rotate(convertAngle(odometer.getWheelRadius(),odometer.getBaseWidth(),correctionAngle),false);
					}
					this.setSpeeds(0, 0);
					if(posY)
					{
						this.odometer.setPosition((new double[] {0,0,90}), (new boolean[] {false,false,true}));
					}
					else
					{
						this.odometer.setPosition((new double[] {0,0,270}), (new boolean[] {false,false,true}));
					}
					try{Thread.sleep(1000);}catch(Exception e){}
					correctHeading = false;
				}
			}
		}
		
		// Stop the robots motion
		this.setSpeeds(0, 0);
		try{Thread.sleep(1000);}catch(Exception e){}
		
		
		// Turn in the positive x direction (0 degrees)
		if(x - odometer.getX() >= 0)
		{
			this.turnTo(0, true);
			posX=true;
		}
		
		// Turn in the negative x direction (270 degrees)
		else
		{
			this.turnTo(180, true);
			posX=false;
		}
		
		
		// Drive forward
		while(Math.abs(x - odometer.getX()) > CM_ERR)
		{
			
			if(getFilteredDataF() < 20)
			{
				Sound.beep();
				if(posX)
				{
					this.turnTo(90, true);
					if(this.odometer.getY() <= 6*TILE_LENGTH)
					{
						while(getFilteredDataR() < 20)
						{
							this.setSpeeds(-SLOW,-SLOW);
						}
						double currentX = this.odometer.getY();
						while(Math.abs(currentX-this.odometer.getY()) < rightSensorToFront)
						{
							this.setSpeeds(-SLOW,-SLOW);
						}
						this.setSpeeds(0,0);
					}
					else
					{
						while(getFilteredDataR() < 20)
						{
							this.setSpeeds(SLOW,SLOW);
						}
						double currentY = this.odometer.getY();
						while(Math.abs(currentY-this.odometer.getY()) < rightSensorToBack)
						{
							this.setSpeeds(SLOW,SLOW);
						}
						this.setSpeeds(0,0);
					}
				}
				else
				{
					this.turnTo(270, true);
					if(this.odometer.getX() <= 6*TILE_LENGTH)
					{
						while(getFilteredDataR() < 20)
						{
							this.setSpeeds(SLOW,SLOW);
						}
						double currentY = this.odometer.getY();
						while(Math.abs(currentY-this.odometer.getY()) < rightSensorToBack)
						{
							this.setSpeeds(SLOW,SLOW);
						}
						this.setSpeeds(0,0);
					}
					else
					{
						while(getFilteredDataR() < 20)
						{
							this.setSpeeds(-SLOW,-SLOW);
						}
						double currentY = this.odometer.getY();
						while(Math.abs(currentY-this.odometer.getY()) < rightSensorToFront)
						{
							this.setSpeeds(-SLOW,-SLOW);
						}
						this.setSpeeds(0,0);
					}
				}
				
				this.travelTo(x, y);
				return;
			}
			else
			{
				if(!correctHeading)
				{
					this.setSpeeds(FAST,FAST);
				}
				else
				{

					this.setSpeeds(0,0);
					try{Thread.sleep(500);}catch(Exception e){}
					this.setSpeeds(SLOW,SLOW);
					if(rightFirst)
					{
						this.leftMotor.rotate(convertAngle(odometer.getWheelRadius(),odometer.getBaseWidth(),correctionAngle),true);
						this.rightMotor.rotate(-convertAngle(odometer.getWheelRadius(),odometer.getBaseWidth(),correctionAngle),false);
					}
					else
					{
						this.leftMotor.rotate(-convertAngle(odometer.getWheelRadius(),odometer.getBaseWidth(),correctionAngle),true);
						this.rightMotor.rotate(convertAngle(odometer.getWheelRadius(),odometer.getBaseWidth(),correctionAngle),false);
					}
					this.setSpeeds(0, 0);
					if(posX)
					{
						this.odometer.setPosition((new double[] {0,0,0}), (new boolean[] {false,false,true}));
					}
					else
					{
						this.odometer.setPosition((new double[] {0,0,180}), (new boolean[] {false,false,true}));
					}
					try{Thread.sleep(1000);}catch(Exception e){}
					correctHeading = false;
				}
			}
		}
		
		// Stop the robots motion
		this.setSpeeds(0, 0);
		try{Thread.sleep(1000);}catch(Exception e){}
		}
	

	/*
	 * TurnTo function which takes an angle and boolean as arguments The boolean controls whether or not to stop the
	 * motors when the turn is completed
	 */
	public void turnTo(double angle, boolean stop) {
		isTurning = true;
		double error = angle - this.odometer.getAng();

		if (error < -180.0) {
			this.setSpeeds(-SLOW, SLOW);
		} else if (error < 0.0) {
			this.setSpeeds(SLOW, -SLOW);
		} else if (error > 180.0) {
			this.setSpeeds(SLOW, -SLOW);
		} else {
			this.setSpeeds(-SLOW, SLOW);
		}
		
		while (Math.abs(error) > DEG_ERR) {

			error = angle - this.odometer.getAng();

		}

		if (stop) {
			this.setSpeeds(0, 0);
		}
		
		try{Thread.sleep(1000);}catch(Exception e){}
		
//		leftMotor.setSpeed(100);
//		rightMotor.setSpeed(100);
//		
//		double error = angle - this.odometer.getAng();
//		
//		if(error < -180)
//		{
//			angle = 360-Math.abs(error);
//			leftMotor.rotate(-convertAngle(this.odometer.getWheelRadius(), this.odometer.getBaseWidth(), angle), true);
//			rightMotor.rotate(convertAngle(this.odometer.getWheelRadius(), this.odometer.getBaseWidth(), angle), false);
//		}
//		else if(error > 180)
//		{
//			angle = 360-Math.abs(error);
//			leftMotor.rotate(convertAngle(this.odometer.getWheelRadius(), this.odometer.getBaseWidth(), angle), true);
//			rightMotor.rotate(-convertAngle(this.odometer.getWheelRadius(), this.odometer.getBaseWidth(), angle), false);
//		}
//		else if(error < 0)
//		{
//			angle = Math.abs(error);
//			leftMotor.rotate(convertAngle(this.odometer.getWheelRadius(), this.odometer.getBaseWidth(), angle), true);
//			rightMotor.rotate(-convertAngle(this.odometer.getWheelRadius(), this.odometer.getBaseWidth(), angle), false);
//		}
//		else
//		{
//			angle = Math.abs(error);
//			leftMotor.rotate(-convertAngle(this.odometer.getWheelRadius(), this.odometer.getBaseWidth(), angle), true);
//			rightMotor.rotate(convertAngle(this.odometer.getWheelRadius(), this.odometer.getBaseWidth(), angle), false);
//		}
//		
//		try{Thread.sleep(1000);}catch(Exception e){}
//		
//		double error = angle - this.odometer.getAng();
//		
//		if (error < -180.0) {
//			while(Math.abs(error) > DEG_ERR)
//			{
//				rotateCCW();
//				error = angle - this.odometer.getAng();
//			}
//		} else if (error < 0.0) {
//			while(Math.abs(error) > DEG_ERR)
//			{
//				rotateCW();
//				error = angle - this.odometer.getAng();
//			}
//		} else if (error > 180.0) {
//			while(Math.abs(error) > DEG_ERR)
//			{
//				rotateCW();
//				error = angle - this.odometer.getAng();
//			}
//		} else {
//			while(Math.abs(error) > DEG_ERR)
//			{
//				rotateCCW();
//				error = angle - this.odometer.getAng();
//			}
//		}
//		this.setSpeeds(0, 0);
//		try{Thread.sleep(1000);}catch(Exception e){}

		isTurning = false;
	}
	
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
	
	/*
	 * Go foward a set distance in cm
	 */
	public void goForward(double distance) {
		this.travelTo(Math.cos(Math.toRadians(this.odometer.getAng())) * distance, Math.cos(Math.toRadians(this.odometer.getAng())) * distance);

	}
	
	// Counter clockwise rotation
	private  void rotateCCW()
	{
		this.setSpeeds(-SLOW, SLOW);
	}

	// Clockwise rotation
	private  void rotateCW()
	{
		this.setSpeeds(SLOW, -SLOW);
	}
	
	public static boolean isTurning(){
		return isTurning;
	}
	
	public void correctHeading(boolean rightFirstTemp, double correctionAng)
	{
		this.correctionAngle = correctionAng;
		this.rightFirst = rightFirstTemp;
		this.correctHeading = true;
	}
	
	private float getFilteredDataR() 
	{
		
		usSensorR.fetchSample(usDataR, 0);
		float distance = usDataR[0]*100;
		int filterValue = 50;
		// If the usSensor reads anything greater then filterValue, set the distance
		// to the filterValue
		if (distance >= filterValue)
		{
			distance = filterValue;
		}
		return distance;
	}
	private float getFilteredDataF() 
	{
		
		usSensorF.fetchSample(usDataF, 0);
		float distance = usDataF[0]*100;
		int filterValue = 50;
		// If the usSensor reads anything greater then filterValue, set the distance
		// to the filterValue
		if (distance >= filterValue)
		{
			distance = filterValue;
		}
		return distance;
	}
}
