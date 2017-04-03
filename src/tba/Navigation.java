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
 * @author	Jordan Itzkovitz
 * @author	William Wang
 * 
 * The navigation class controls how the robot will maneuver about the grid.
 * The robot navigates along the x and y axes only, thus at no angles other
 * than 0 degrees, 90 degrees, 180 degrees and 270 degrees. Obstacle avoidance
 * is also performed in this class.*/

public class Navigation {
	final static int FAST = 200, SLOW = 150, ACCELERATION = 1000, ACCELERATION_SLOW = 1000;
	final static double DEG_ERR = 2.5, CM_ERR = 1.0, TILE_LENGTH = 30.48, rightSensorToBack = 23, rightSensorToFront = 15;
	private Odometer odometer;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	private boolean isTurning = false;
	private boolean rightFirst, avoiding = false, correctHeading = false, stopAvoiding = false;
	private double correctionAngle = 0;
	private final int filterValueR = 45;
	private final int filterValueF = 25;
	SampleProvider usSensorR; 
	float[] usDataR;
	SampleProvider usSensorF; 
	float[] usDataF;
	SampleProvider colorSensorR; 
	float[] colorDataR;
	SampleProvider colorSensorL; 
	float[] colorDataL;
	private boolean forward = false;
	private boolean defense = false;
	private int boardDimensions;
	private boolean collecting = false;
	private double shootingDistance;

	/**
	 * Navigation constructor class
	 * @param 	odo 	Odometer object in charge of providing Odometer class methods
	 * @param 	nav 	Navigation object in charge of providing Navigation class methods
	 * @param 	colorSensorR 	Sample provider variable for the right color sensor
	 * @param 	colorSensorL 	Sample provider variable for the left color sensor
	 * @param 	colorDataR 	Float array to hold right color sensor data
	 * @param 	colorDataL 	Float array to hold left color sensor data
	 * @param 	usSensorR 	Sample provider variable for the us sensor located on the right side of the robot
	 * @param 	usSensorF 	Sample provider variable for the us sensor located at the front of the robot
	 * @param 	usDataR 	Float array to hold right us sensor data
	 * @param 	usDataR 	Float array to hold front us sensor data
	 * @param 	boardDimensions 	The dimensions of the given board (assuming square board)
	 * @param 	shootingDistance 	The shooting line distance from the hoop
	 * */
	
	public Navigation(Odometer odo, SampleProvider usSensorR, float[] usDataR, SampleProvider usSensorF,
					float[] usDataF, int boardDimensions,SampleProvider colorSensorR, float[] colorDataR,
					SampleProvider colorSensorL, float[] colorDataL, double shootingDistance) {
		
		this.odometer = odo;
		EV3LargeRegulatedMotor[] motors = this.odometer.getMotors();
		this.leftMotor = motors[0];
		this.rightMotor = motors[1];
		this.usSensorR = usSensorR;
		this.usDataR = usDataR;
		this.usSensorF = usSensorF;
		this.usDataF = usDataF;
		this.colorSensorR = colorSensorR;
		this.colorDataR = colorDataR;
		this.colorSensorL = colorSensorL;
		this.colorDataL = colorDataL;
		this.boardDimensions = boardDimensions/2;
		this.shootingDistance = shootingDistance;
		this.leftMotor.setAcceleration(ACCELERATION);
		this.rightMotor.setAcceleration(ACCELERATION);
	}

	/**
	 * Sets the motor speeds and sets either forward or backward motion based on the sign of the 
	 * value passed
	 * 
	 * @param 	lSpd 	Left motor speed (int)
	 * @param 	rSpd 	Right motor speed (int)
	 * 
	 * @return 		void
	 * */
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

	/**
	 * Sets the motor speeds and sets either forward or backward motion based on the sign of the 
	 * value passed
	 * 
	 * @param 	lSpd 	Left motor speed (float)
	 * @param 	rSpd 	Right motor speed (float)
	 * 
	 * @return 		void
	 * */
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

	/**
	 * Float the motors
	 * 
	 * @return 		void
	 * */
	public void setFloat() {
		this.leftMotor.stop();
		this.rightMotor.stop();
		this.leftMotor.flt(true);
		this.rightMotor.flt(true);
	}

	/**
	 * Calls realTravelTo() to send the robot to a given x and y point. This method will call realTravelTo()
	 * in a way to avoid the defensive/bounce zones for offense and defense respectively
	 * 
	 * @param 	x 	X point to travel to
	 * @param 	y 	Y point to travel to
	 * 
	 * @return 		void
	 * */
	public void travelTo(double x, double y){
		if(forward)
		{
			double avoidZone = 2*boardDimensions*TILE_LENGTH-shootingDistance;
			if((y > avoidZone)&& !((x<TILE_LENGTH&&odometer.getX()<TILE_LENGTH)||
					(x>(boardDimensions - 1)*TILE_LENGTH&&odometer.getX()<(boardDimensions - 1)*TILE_LENGTH)))
			{
				realTravelTo(x,avoidZone-TILE_LENGTH/2);
				realTravelTo(x,y);
			}
			else
			{
				realTravelTo(x,y);
			}
		}


	}
	
	/**
	 * Sends the robot to a given x and y point, always travling in the y driection first, followed
	 * by the x direction. This method also handles obstacle avoidance.
	 * 
	 * @param 	x 	X point to travel to
	 * @param 	y 	Y point to travel to
	 * 
	 * @return 		void
	 * */
	public void realTravelTo(double x, double y) {
		
		//Set motors acceleration value
		this.leftMotor.setAcceleration(ACCELERATION_SLOW);
		this.rightMotor.setAcceleration(ACCELERATION_SLOW);
		
		//Hold information of whether the robot is traveling in the y or x directions
		boolean posY,posX;

		//x and y values which will be used throughout this method
		double currentX = 0;
		double currentY = 0;
		
		/* The navigation will work as follows: the robot will travel in the y direction followed by the
		 * x direction. This is done in order to more easily correct odometry with the light sensor as 
		 * the robot will only be traveling along the x and y axes. The us sensor located at the front
		 * of the robot will also continuously poll data to track obstacles and avoid them.*/
		
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
		
		// Travel in the y direction
		while(Math.abs(y - odometer.getY()) > CM_ERR)
		{
			currentX = odometer.getX();
			
			/*If the front us sensor sees an obstacle, is not turning, is not in the dispenser zone and is not
			 * detecting the boarders, then perform obstacle avoidance. Otherwise, keep on traveling to the given y point
			 * */
			if(getFilteredDataF() < filterValueF && !isTurning() && !isInDispenserZone() && !isDetectingBorder())
			{
				avoiding = true;
				Sound.beep();
				
				/*If travling in the positive y direction, turn to 180 degrees so that the right us sensor
				 * is facing the obstacle. Otherwise, turn to 0 degrees
				 * */
				if(posY)
				{
					this.turnTo(180, true);
					/*If the robot is in the left half of the board, avoid the obstacle by going to the right,
					 * and otherwise avoid in the left.
					 * */
					if(this.odometer.getX() <= boardDimensions*TILE_LENGTH)
					{
						avoid(true,false,-SLOW);
						
						turnTo(270,true);
						
						avoid(false,false,-SLOW);
					}
					else
					{
						avoid(true,true,SLOW);
						
						turnTo(90,true);
						
						avoid(false,true,SLOW);
					}
				}
				
				//Negative y direction
				else
				{
					this.turnTo(0, true);

					/*If the robot is in the left half of the board, avoid the obstacle by going to the right,
					 * and otherwise avoid in the left.
					 * */
					if(this.odometer.getX() <= boardDimensions*TILE_LENGTH)
					{
						avoid(true,true,SLOW);
						
						turnTo(270,true);
						
						avoid(false,true,SLOW);
					}
					else
					{
						avoid(true,false,-SLOW);
						
						turnTo(90,true);
						
						avoid(false,false,-SLOW);
					}
				}
				avoiding = false;
				stopAvoiding = false;
				
				//Recall travel to from the new x and y positions
				this.travelTo(x, y);
				return;
			}
			else
			{
				/*If the correctHeading thread has not detected a line (hence has not set correctHeading
				 * to true), continue the robots motion. Otherwise, correct the heading accordingly.
				 */
				if(!correctHeading)
				{
					this.setSpeeds(FAST,FAST);
				}
				else
				{
					rest(500);
					
					this.setSpeeds(SLOW,SLOW);
					Sound.twoBeeps();
					
					//If the right sensor hits a black line first turn clockwise, and otherwise turn counterclockwise
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
	
					//Reset the odometers heading angle based on whether the robot is traveling in positive Y or not
					if(posY)
					{
						this.odometer.setPosition((new double[] {0,0,90}), (new boolean[] {false,false,true}));
					}
					else
					{
						this.odometer.setPosition((new double[] {0,0,270}), (new boolean[] {false,false,true}));
					}
					rest(1000);
					correctHeading = false;
				}
			}
		}
		
		rest(1000);
		
		//Turn in the positive x direction (0 degrees)
		if(x - odometer.getX() >= 0)
		{
			this.turnTo(0, true);
			posX=true;
		}
		
		//Turn in the negative x direction (270 degrees)
		else
		{
			this.turnTo(180, true);
			posX=false;
		}
		
		
		//Travel in the x direction
		while(Math.abs(x - odometer.getX()) > CM_ERR)
		{
			/*If the front us sensor sees an obstacle, is not turning, is not in the dispenser zone and is not
			 * detecting the boarders, then perform obstacle avoidance. Otherwise, keep on traveling to the given y point
			 * */
			if(getFilteredDataF() < filterValueF && !isTurning() && !isInDispenserZone() && !isDetectingBorder())
			{
				correctHeading = false;
				avoiding = true;
				Sound.beep();
				
				/*If travling in the positive x direction, turn to 90 degrees so that the right us sensor
				 * is facing the obstacle. Otherwise, turn to 270 degrees
				 * */
				if(posX)
				{
					this.turnTo(90, true);
			
					/*If the robot is in the bottom half of the board, avoid the obstacle by going up the board,
					 *and otherwise avoid by going down the board.
					 * */
					if(this.odometer.getY() <= boardDimensions*TILE_LENGTH)
					{
						avoid(false,true,SLOW);
						
						
						/*Avoiding an obstacle works differently then traveling in the x direction due to the fact that realTravelTo()
						 * always travels in the y direction first. After avoiding the obstacle by traveling in the y direction
						 * the robot must also avoid the obstacle in the x direction, because otherwise recalling travelTo()
						 * will send the robot back to the y position that it just moved away from, and this will continuously occur.
						 * Thus, avoiding in the y direction followed by the x direction must be done. This works the same
						 * as (*), and this is used in throughout the rest of the method, so call this block (**).
						 * */
						this.turnTo(0,true);
						
						avoid(true,true,SLOW);
					}
					else
					{
						avoid(false,false,-SLOW);
						
						this.turnTo(180,true);
						
						avoid(true,false,-SLOW);
					}
				}
				
				//Negative x direction
				else
				{
					this.turnTo(270, true);

					/*If the robot is in the bottom half of the board, avoid the obstacle by going up the board,
					 *and otherwise avoid by going down the board.
					 * */
					if(this.odometer.getY() <= boardDimensions*TILE_LENGTH)
					{
						avoid(false,false,-SLOW);

						this.turnTo(0,true);

						avoid(true,false,-SLOW);
					}
					else
					{

						avoid(false,true,SLOW);

						this.turnTo(180,true);
						
						avoid(true,true,SLOW);
					}
					avoiding = false;
					stopAvoiding = false;
					this.travelTo(x, y);
					return;
				}
			}
			else
			{
				
				/*If the correctHeading thread has not detected a line (hence has not set correctHeading
				 * to true), continue the robots motion. Otherwise, correct the heading accordingly.
				 */
				if(!correctHeading)
				{
					this.setSpeeds(FAST,FAST);
				}
				else
				{

					rest(500);
					
					this.setSpeeds(SLOW,SLOW);
					Sound.twoBeeps();
					
					//If the right sensor hits a black line first turn clockwise, and otherwise turn counterclockwise
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
					
					//Reset the odometers heading angle based on whether the robot is traveling in the positive x direction or not
					if(posX)
					{
						this.odometer.setPosition((new double[] {0,0,0}), (new boolean[] {false,false,true}));
					}
					else
					{
						this.odometer.setPosition((new double[] {0,0,180}), (new boolean[] {false,false,true}));
					}
					rest(1000);
					correctHeading = false;
				}
			}
		}

		rest(1000);
		}
	
	/**
	 * Takes an angle and boolean as arguments and turns to a specific angle. 
	 * The boolean controls whether or not to stop the motors when the turn is completed
	 * 
	 * @param 	angle 	Angle to turn to
	 * @param 	stop 	Boolean to rest motors
	 * 
	 * @return		void
	 * */
	public void turnTo(double angle, boolean stop) {
		
		//Set isTurning to true so that correct heading and obstacle avoidance know that the robot is turning
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
		
		//Set isTurning to false
		isTurning = false;
	}
	
	/**
	 * Avoid obstacles based on where the robot is located on the board
	 * 
	 * @param	xDirection	States whether the robot is moving in the x direction or not
	 * @param	pos	States whether robot is moving forward or not
	 * @param	speed	Speed that the robot should avoid at
	 * 
	 * @return		void
	 * */
	private void avoid(boolean xDirection, boolean pos, int speed)
	{
		/*The following three while loops perform three important tasks.
		 * The first waits until the obstacle is seen by the right us sensor.
		 * The second waits until the obstacle is unseen by the right us sensor.
		 * The third travels the remaining distance between the sensor and either
		 * the front or back of the robot based on whether it is traveling backwards
		 * or forward, respectively. This same function will be repeated throughout
		 * this method so reference this as
		 * */
		
		if(xDirection)
		{
			double currentX = odometer.getX();
			while(getFilteredDataR() > filterValueR && !stopAvoiding)
			{
				if(!correctHeading)
				{
					stillFollowing(pos,xDirection,currentX);
				}
				else{
					doCorrectHeading(rightFirst, correctionAngle, pos);
				}

			}
			while(getFilteredDataR() < filterValueR && !stopAvoiding)
			{

				if(!correctHeading)
				{
					this.setSpeeds(speed,speed);
				}
				else{
					doCorrectHeading(rightFirst, correctionAngle, pos);
				}
			}
			currentX = this.odometer.getX();
			while(Math.abs(currentX-this.odometer.getX()) < rightSensorToFront && !stopAvoiding)
			{
				if(!correctHeading)
				{
					this.setSpeeds(speed,speed);
				}
				else{
					doCorrectHeading(rightFirst, correctionAngle, pos);
				}
			}
		}
		else
		{
			double currentY = odometer.getY();
			while(getFilteredDataR() > filterValueR && !stopAvoiding)
			{
				if(!correctHeading)
				{
					stillFollowing(pos,xDirection,currentY);
				}
				else{
					doCorrectHeading(rightFirst, correctionAngle, pos);
				}

			}
			while(getFilteredDataR() < filterValueR && !stopAvoiding)
			{

				if(!correctHeading)
				{
					this.setSpeeds(speed,speed);
				}
				else{
					doCorrectHeading(rightFirst, correctionAngle, pos);
				}
			}
			currentY = this.odometer.getY();
			while(Math.abs(currentY-this.odometer.getY()) < rightSensorToFront && !stopAvoiding)
			{
				if(!correctHeading)
				{
					this.setSpeeds(speed,speed);
				}
				else{
					doCorrectHeading(rightFirst, correctionAngle, pos);
				}
			}
		}
		
		rest(500);
	}
	
	/**
	 * Convert distance to degrees
	 * 
	 * @param 	radius 	Radius of the robot
	 * @param 	distance 	Distance of rotation in radians
	 * 
	 * @return 		Converted distance in cm to degrees
	 * */
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	/**
	 * Convert Angle to degrees
	 * 
	 * @param 	radius 	Radius of the robot
	 * @param 	distance 	Distance of rotation in radians
	 * 
	 * @return 		Distance for wheels to turn in degrees
	 * */
	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
	
	/**
	 * Drive the robot a set distance and direction
	 * 
	 * @param 	distance 	Distance to travel
	 * @param 	xDirection 	Boolean to state whether the robot is traveling in the x direction or not
	 * @param 	speed 	Speed for travel
	 * @param 	pos 	Boolean to state whether to travel forward or backward
	 * 
	 * @return 		void
	 * */
	public void drive(double distance, boolean xDirection, int speed, boolean pos) {
		
		if(pos)
		{
			if(xDirection)
			{
				double currentX = odometer.getX();
				while(Math.abs(currentX-odometer.getX()) < distance)
				{
					this.setSpeeds(speed,speed);
				}
			}
			else
			{
				double currentY = odometer.getY();
				while(Math.abs(currentY-odometer.getY()) < distance)
				{
					this.setSpeeds(speed,speed);
				}
			}
		}
		else
		{
			if(xDirection)
			{
				double currentX = odometer.getX();
				while(Math.abs(currentX-odometer.getX()) < distance)
				{
					this.setSpeeds(-speed,-speed);
				}
			}
			else
			{
				double currentY = odometer.getY();
				while(Math.abs(currentY-odometer.getY()) < distance)
				{
					this.setSpeeds(-speed,-speed);
				}
			}
		}
		
		this.setSpeeds(0,0);
	}

	/**
	 * Checks if the robot is turning
	 * 
	 * @return 		isTurning boolean
	 * */
	public boolean isTurning(){
		return isTurning;
	}
	
	/**
	 * Checks if the robot is avoiding
	 * 
	 * @return 		avoiding boolean
	 * */
	public boolean isAvoiding()
	{
		return avoiding;
	}
	
	/**
	 * Sets the appropriate booleans and global variables in order for the robot to correct its heading
	 * 
	 * @param 	rightFirstTemp 	States whether the right color sensor hit the line first or not
	 * @param 	correctionAng 	Angle for correction
	 * 
	 * @return 		void
	 * */
	public void correctHeading(boolean rightFirstTemp, double correctionAng)
	{
		this.correctionAngle = correctionAng;
		this.rightFirst = rightFirstTemp;
		this.correctHeading = true;
	}
	
	/**
	 * This method essentially checks if the robot is avoiding "nothing". For example, if the robot senses
	 * an obstacle and turns to the side of the right sensor but the right sensor is already
	 * passed the obstacle, the robot will drive a maximum of 1.5 tiles before stopping
	 * avoidance. If this check is not done, the robot will continue to travel until an obstacle is seen,
	 * which may be never and thus will crash. This method also handles the driving up until the maximum
	 * distance is reached.
	 * 
	 * @param 	forward 	States whether the robot should travel forward or not
	 * @param 	xDirection 	States whether the robot is traveling in the x direction or not
	 * @param 	point 	The starting point of travel to mark how much the robot has traveled since the avoidance began
	 * 
	 * @return 		void
	 * */
	private void stillFollowing(boolean forward, boolean xDirection, double point)
	{
		if(forward)
		{
			if(xDirection)
			{
				if(Math.abs(point-odometer.getX()) > TILE_LENGTH)
				{
					stopAvoiding = true;
				}
				else if(!stopAvoiding)
				{
					this.setSpeeds(SLOW, SLOW);
				}
			}
			else
			{
				if(Math.abs(point-odometer.getY()) > TILE_LENGTH)
				{
					stopAvoiding = true;
				}
				else if(!stopAvoiding)
				{
					this.setSpeeds(SLOW, SLOW);
				}
			}
		}
		else
		{
			if(xDirection)
			{
				if(Math.abs(point-odometer.getX()) > TILE_LENGTH)
				{
					stopAvoiding = true;
				}
				else if(!stopAvoiding)
				{
					this.setSpeeds(-SLOW, -SLOW);
				}
			}
			else
			{
				if(Math.abs(point-odometer.getY()) > TILE_LENGTH)
				{
					stopAvoiding = true;
				}
				else if(!stopAvoiding)
				{
					this.setSpeeds(-SLOW, -SLOW);
				}
			}
		}
	}
	
	/**
	 * Get the the us Sensor distance value from the right us sensor
	 * 
	 * @return 		Right us sensor value
	 * */
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
	
	/**
	 * Get the the us Sensor distance value from the front us sensor
	 * 
	 * @return 		Front us sensor value
	 * */
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
	
	/**
	 * Do the appropriate heading correction based on the calculate values passed from the correctHeading
	 * thread
	 * 
	 * @param 	rightFirst 	States whether the right sensor hit the black line first or not
	 * @param 	correctionAngle 	Angle for correction
	 * @param 	posDirection 	States whether the correction will be done while driving forward or not
	 * 
	 * @return 		void
	 * */
	private void doCorrectHeading(boolean rightFirst, double correctionAngle, boolean posDirection)
	{
		rest(500);
		Sound.twoBeeps();
		
		if(posDirection)
		{
			//If rightFirst rotate clockwise, else counterclockwise
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
		}
		else
		{
			//While traveling backwards and correcting, backup a set distance in order for the light sensors to not re-sense the past line
			if((odometer.getAng() > 355 || odometer.getAng() < 5) || (odometer.getAng() > 175 && odometer.getAng() < 185))
			{
				drive(5,true,SLOW,false);
			}
			else
			{
				drive(5,false,SLOW,false);
			}
			
			rest(500);

			//If rightFirst rotate counterclockwise, else clockwise
			if(rightFirst)
			{
				this.leftMotor.rotate(-convertAngle(odometer.getWheelRadius(),odometer.getBaseWidth(),correctionAngle),true);
				this.rightMotor.rotate(convertAngle(odometer.getWheelRadius(),odometer.getBaseWidth(),correctionAngle),false);
				Sound.buzz();
			}
			else
			{
				this.leftMotor.rotate(convertAngle(odometer.getWheelRadius(),odometer.getBaseWidth(),correctionAngle),true);
				this.rightMotor.rotate(-convertAngle(odometer.getWheelRadius(),odometer.getBaseWidth(),correctionAngle),false);
				Sound.buzz();
			}	
		}
		this.setSpeeds(0, 0);
		correctHeading = false;
	}
	
	
	/**
	 * States whether the robot is the forward team
	 * 
	 * @return 		void
	 * */
	public void forwardTeam(){
		this.forward = true;
		this.defense = false;
	}
	
	/**
	 * States whether the robot is the defensive team
	 * 
	 * @return 		void
	 * */
	public void defenseTeam(){
		this.defense = true;
		this.forward = true;
	}
	
	/**
	 * Light localization at the dispenser. The robot will always be facing the x direction once it reaches
	 * its destination, thus we travel in the x until a line is crossed, correct the robots heading, and then
	 * do the same in the y direction depending on the dispensers orientation. The way in which correction is done
	 * in this method is very similar to LightLocalizerV4, however minor changes had to be made to suit this
	 * exact situation.
	 * 
	 * @return 		void
	 * */
	public void dispenserLocalization()
	{
		this.collecting = true;
		double minLight =0.3;
		
		// Travel in the x direction until a black line is seen by both sensors
		boolean rightHit = false,leftHit = false;
		double firstHit = 0;
		double correction = 0;
		
		// travel in the x direction until both light sensors sense a black line, and perform correction
		if(odometer.getX()<TILE_LENGTH){
			this.turnTo(180, true);
		}
		while(true)
		{
			this.setSpeeds(SLOW,SLOW);
			if(getColorDataR()<minLight)
			{
				if(leftHit)
				{
					correction = this.odometer.getX()-firstHit+.5;
					correction = Math.toDegrees(Math.asin(correction/odometer.getBaseWidth()));
					doCorrectHeading(false, correction, true);
					leftHit=false;
					rightHit=false;
					break;
				}
				else{
					rightHit = true;
					firstHit = this.odometer.getX();
				}
			}

			if(getColorDataL()<minLight)
			{

				if(rightHit)
				{
					correction = this.odometer.getX()-firstHit+.5;
					correction = Math.toDegrees(Math.asin(correction/odometer.getBaseWidth()));
					doCorrectHeading(true, correction, true);
					leftHit=false;
					rightHit=false;
					break;
				}
				else{
					leftHit = true;
					firstHit = this.odometer.getX();
				}
			}
		}
		
		rest(500);
		
		drive(LightLocalizerV4.lightSensorDistance,true,SLOW,true);
		
		rest(500);
		
		// Turn to appropriate y direction and travel until both sensors sense a black line and perform correction
		if(odometer.getY()<TILE_LENGTH){
			this.turnTo(270,true);
		}
		else{
			this.turnTo(90,true);
		}
		
			while(true)
			{
				this.setSpeeds(SLOW,SLOW);
				if(getColorDataR()<minLight)
				{
					if(leftHit)
					{
						correction = this.odometer.getY()-firstHit+.5;
						correction = Math.toDegrees(Math.asin(correction/odometer.getBaseWidth()));
						doCorrectHeading(false,correction,true);
						leftHit=false;
						rightHit=false;
						break;
					}
					else{
						rightHit = true;
						firstHit = this.odometer.getY();
					}
				}

				if(getColorDataL()<minLight)
				{

					if(rightHit)
					{
						correction = this.odometer.getY()-firstHit+.5;
						correction = Math.toDegrees(Math.asin(correction/odometer.getBaseWidth()));
						doCorrectHeading(true,correction,true);
						leftHit=false;
						rightHit=false;
						break;
					}
					else{
						leftHit = true;
						firstHit = this.odometer.getY();
					}
				}
			}
			
			rest(500);
			drive(LightLocalizerV4.lightSensorDistance,false,SLOW,true);
			this.collecting = false;
			rest(500);
	}
	
	
	/**
	 * Turn a certain number of degrees
	 * 
	 * @param 	cw 	States whether the robot should turn clockwise or not
	 * @param 	angle 	Set angle for turn
	 * 
	 * @return 		void
	 * */
	private void turn(boolean cw, double angle)
	{
		if(cw)
		{
			this.leftMotor.rotate(convertAngle(odometer.getWheelRadius(),odometer.getBaseWidth(),angle),true);
			this.rightMotor.rotate(-convertAngle(odometer.getWheelRadius(),odometer.getBaseWidth(),angle),false);
		}
		else
		{
			this.leftMotor.rotate(-convertAngle(odometer.getWheelRadius(),odometer.getBaseWidth(),angle),true);
			this.rightMotor.rotate(convertAngle(odometer.getWheelRadius(),odometer.getBaseWidth(),angle),false);
		}
	}
	
	/**
	 * States whether the robot is in the ball dispenser zone or not
	 * 
	 * @return 		Boolean stating whether the robot is in the dispenser zone or not
	 * */
	private boolean isInDispenserZone()
	{
		if((odometer.getX() > Play.dispX-TILE_LENGTH && odometer.getX() < Play.dispX+TILE_LENGTH) && (odometer.getY() > Play.dispY-TILE_LENGTH && odometer.getY() < Play.dispY+TILE_LENGTH))
		{
			return true;
		}
		return false;
	}
	
	/**
	 * States whether the robot is detecting one of the boarder walls or not
	 * 
	 * @return 		Boolean whether the robot is detecting one of the boarder walls or not
	 * */
	private boolean isDetectingBorder()
	{
		if(odometer.getX()<TILE_LENGTH/2 && (odometer.getAng() > 170 && odometer.getAng() < 190))
		{
			return true;
		}
		else if(odometer.getX()>2*boardDimensions*TILE_LENGTH-TILE_LENGTH/2 && (odometer.getAng() > 350 || odometer.getAng() < 10))
		{
			return true;
		}
		else if(odometer.getY()>2*boardDimensions*TILE_LENGTH-TILE_LENGTH/2 && (odometer.getAng() > 80 && odometer.getAng() < 100))
		{
			return true;
		}
		else if(odometer.getY()<TILE_LENGTH/2 && (odometer.getAng() > 260 && odometer.getAng() < 280))
		{
			return true;
		}
		
		return false;
	}
	
	/**
	 * Get the the light strength from the right color sensor
	 * 
	 * @return 		Right color sensor value
	 * */
	public float getColorDataR(){
		colorSensorR.fetchSample(colorDataR, 0);
		float lightStrength = colorDataR[0];
		return lightStrength;
	}
	
	/**
	 * Get the the light strength from the left color sensor
	 * 
	 * @return 		Left color sensor value
	 * */
	public float getColorDataL(){
		colorSensorL.fetchSample(colorDataL, 0);
		float lightStrength = colorDataL[0];
		return lightStrength;
	}
	
	/**
	 * States whether the robot is collecting the ball or not
	 * 
	 * @return 		Boolean stating whether the robot is collecting the ball or not
	 * */
	public boolean isCollecting(){
		return this.collecting;
	}

	/**
	 * Let the robot rest for millis milliseconds by stopping the motors and sleeping the thread
	 * 
	 * @param 	millis Milliseconds wanted for rest period
	 * 
	 * @return		void
	 * */
	public void rest(int millis)
	{
		this.setSpeeds(0,0);
		try{Thread.sleep(millis);}catch(Exception e){}
	}
	
}