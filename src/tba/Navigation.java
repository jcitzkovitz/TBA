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
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * The navigation class controls how the robot will maneuver about the grid.
 * The robot navigates along the x and y axes only, thus at no angles other
 * than 0 degrees, 90 degrees, 180 degrees and 270 degrees.*/

public class Navigation {
	final static int FAST = 150, SLOW = 100, ACCELERATION = 1000, ACCELERATION_SLOW = 1000;
	final static double DEG_ERR = 2.5, CM_ERR = 1.0;
	private Odometer odometer;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	private static boolean isTurning = false;
	private static boolean rightFirst, correctHeading = false;
	private static double xCorrectAng;
	private static double yCorrectAng;

	public Navigation(Odometer odo) {
		this.odometer = odo;

		EV3LargeRegulatedMotor[] motors = this.odometer.getMotors();
		this.leftMotor = motors[0];
		this.rightMotor = motors[1];

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
		// The commented code below is the original navigation code given by the TAs
		/*while (Math.abs(x - odometer.getX()) > CM_ERR || Math.abs(y - odometer.getY()) > CM_ERR) {
			minAng = (Math.atan2(y - odometer.getY(), x - odometer.getX())) * (180.0 / Math.PI);
			if (minAng < 0)
				minAng += 360.0;
			this.turnTo(minAng, false);
			this.setSpeeds(FAST, FAST);
		}*/
		
		/* The navigation will work as follows: the robot will travel in the y direction followed by the
		 * x direction. This is done in order to more easily correct odometry with the light sensor as 
		 * the robot will only be traveling along the x and y axes*/
		
		// Turn in the direction of the positive y axis (90 degrees)

		if(y - odometer.getY() >= 0)
		{
			this.turnTo(90, true);
		}
		
		// Turn in the direction of the negative y axis (270 degrees)
		else
		{
			this.turnTo(270, true);
		}
		
		
		while(Math.abs(y - odometer.getY()) > CM_ERR)
		{
			if(!correctHeading)
			{
				this.setSpeeds(FAST,FAST);
			}
			else
			{
				this.setSpeeds(0,0);
				try{Thread.sleep(500);}catch(Exception e){}
				if(rightFirst)
				{
					this.leftMotor.rotate(convertAngle(odometer.getWheelRadius(),odometer.getBaseWidth(),yCorrectAng),true);
					this.rightMotor.rotate(-convertAngle(odometer.getWheelRadius(),odometer.getBaseWidth(),yCorrectAng),false);
				}
				else
				{
					this.leftMotor.rotate(-convertAngle(odometer.getWheelRadius(),odometer.getBaseWidth(),yCorrectAng),true);
					this.rightMotor.rotate(convertAngle(odometer.getWheelRadius(),odometer.getBaseWidth(),yCorrectAng),false);
				}
				Sound.beep();
				OdometerCorrectionV2.doCorrection();
				try{Thread.sleep(1000);}catch(Exception e){}
				correctHeading = false;
			}
		}
		
		// Stop the robots motion
		this.setSpeeds(0, 0);
		try{Thread.sleep(1000);}catch(Exception e){}
		
		
		// Turn in the positive x direction (0 degrees)
		if(x - odometer.getX() >= 0)
		{
			this.turnTo(0, true);
		}
		
		// Turn in the negative x direction (270 degrees)
		else
		{
			this.turnTo(180, true);
		}
		
		
		// Drive forward
		while(Math.abs(x - odometer.getX()) > CM_ERR)
		{
			if(!correctHeading)
			{
				this.setSpeeds(FAST,FAST);
			}
			else
			{
				
				this.setSpeeds(0,0);
				try{Thread.sleep(500);}catch(Exception e){}
				if(rightFirst)
				{
					this.leftMotor.rotate(convertAngle(odometer.getWheelRadius(),odometer.getBaseWidth(),xCorrectAng),true);
					this.rightMotor.rotate(-convertAngle(odometer.getWheelRadius(),odometer.getBaseWidth(),xCorrectAng),false);
				}
				else
				{
					this.leftMotor.rotate(-convertAngle(odometer.getWheelRadius(),odometer.getBaseWidth(),xCorrectAng),true);
					this.rightMotor.rotate(convertAngle(odometer.getWheelRadius(),odometer.getBaseWidth(),xCorrectAng),false);
				}
				OdometerCorrectionV2.doCorrection();
				try{Thread.sleep(1000);}catch(Exception e){}
				correctHeading = false;
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
	
	public static void correctHeading(boolean rightFirstTemp, double xAng, double yAng)
	{
		correctHeading = true;
		rightFirst = rightFirstTemp;
		xCorrectAng = xAng;
		yCorrectAng = yAng;
	}
}
