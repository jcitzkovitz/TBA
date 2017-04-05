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
	final static int FAST = 200, SLOW = 150, ACCELERATION = 1000, ACCELERATION_SLOW = 1000;
	final static double DEG_ERR = 2.5, CM_ERR = 1.0, TILE_LENGTH = 30.48, rightSensorToBack = 27, rightSensorToFront = 15;
	private Odometer odometer;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	private boolean isTurning = false;
	private boolean rightFirst, avoiding = false, correctHeading = false, stopAvoiding = false;
	private double correctionAngle = 0;
	private final int filterValueR = 30;
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
	private double targetX;
	private double targetY;
	private double w1;
	private double w2;

	public Navigation(Odometer odo, SampleProvider usSensorR, float[] usDataR, SampleProvider usSensorF,
					float[] usDataF, int boardDimensions,SampleProvider colorSensorR, float[] colorDataR,
					SampleProvider colorSensorL, float[] colorDataL, double shootingDistance, double w1, double w2) {
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
		this.w1 = w1;
		this.w2 = w2;

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
	public void travelTo(double x, double y){
		if(forward){
			
			if((y > 2*boardDimensions*TILE_LENGTH-shootingDistance)&&                 !((x<TILE_LENGTH&&odometer.getX()<TILE_LENGTH)||
					(x>(2*boardDimensions - 1)*TILE_LENGTH&&odometer.getX()>(2*boardDimensions - 1)*TILE_LENGTH)))
			{
				realTravelTo(x,2*boardDimensions*TILE_LENGTH-shootingDistance-TILE_LENGTH/2);
				realTravelTo(x,y);
			}
			else
			{
				realTravelTo(x,y);
			}
	
			}	
		else if(defense){
				realTravelTo(x,y);
		}
		
		}
	
	public void realTravelTo(double x, double y) {
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

		
		// Travel in the y direction
				while(Math.abs(y - odometer.getY()) > CM_ERR)
				{
					
					/*If the front us sensor sees an obstacle, is not turning, is not in the dispenser zone and is not
					 * detecting the boarders, then perform obstacle avoidance. Otherwise, keep on traveling to the given y point
					 * */
					if(getFilteredDataF() < filterValueF && !isTurning() && !isInDispenserZone() && !isDetectingBorder())
					{
						Sound.beep();
						
						/*If travling in the positive y direction, turn to 180 degrees so that the right us sensor
						 * is facing the obstacle. Otherwise, turn to 0 degrees
						 * */
						targetX = odometer.getX();
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
						stopAvoiding = false;
						
						//Travel back to the x position you were at initially
						this.realTravelTo(targetX,odometer.getY());
						
						//Recall travel to from the new x and y positions
						this.travelTo(x,y);
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
							this.setSpeeds(0,0);
							try{Thread.sleep(1000);}catch(Exception e){}
							
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
							this.setSpeeds(0,0);
							try{Thread.sleep(1000);}catch(Exception e){}
							correctHeading = false;
						}
					}
					
					
				}
				
				this.setSpeeds(0,0);
				try{Thread.sleep(1000);}catch(Exception e){}
				
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
						Sound.beep();
						
						/*If travling in the positive x direction, turn to 90 degrees so that the right us sensor
						 * is facing the obstacle. Otherwise, turn to 270 degrees
						 * */
						
						targetY = odometer.getY();
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
							stopAvoiding = false;
							
							this.realTravelTo(odometer.getX(),targetY);
							this.travelTo(x,y);
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

							this.setSpeeds(0,0);
							try{Thread.sleep(1000);}catch(Exception e){}
							
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
							this.setSpeeds(0,0);
							try{Thread.sleep(1000);}catch(Exception e){}
							correctHeading = false;
						}
					}
				}
				this.setSpeeds(0,0);
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
	
	public boolean isTurning(){
		return isTurning;
	}
	
	public boolean isAvoiding()
	{
		return avoiding;
	}
	
	public void correctHeading(boolean rightFirstTemp, double correctionAng)
	{
		this.correctionAngle = correctionAng;
		this.rightFirst = rightFirstTemp;
		this.correctHeading = true;
	}
	
	private void stillFollowing(boolean forward, boolean xDirection, double point)
	{
		if(forward)
		{
			if(xDirection)
			{
				if(Math.abs(point-odometer.getX()) > TILE_LENGTH*3/2)
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
				if(Math.abs(point-odometer.getY()) > TILE_LENGTH*3/2)
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
				if(Math.abs(point-odometer.getX()) > TILE_LENGTH*3/2)
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
				if(Math.abs(point-odometer.getY()) > TILE_LENGTH*3/2)
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
	
	
	
	
	private void doCorrectHeading(boolean rightFirst, double correctionAngle, boolean posDirection)
	{
		this.setSpeeds(0,0);
		try{Thread.sleep(500);}catch(Exception e){}

		Sound.twoBeeps();
		if(posDirection)
		{
			this.leftMotor.setSpeed(SLOW);
			this.rightMotor.setSpeed(SLOW);
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
			if((odometer.getAng() > 355 || odometer.getAng() < 5) || (odometer.getAng() > 175 && odometer.getAng() < 185))
			{
				double currentX = odometer.getX();
				while(Math.abs(currentX-odometer.getX()) < 5)
				{
					this.setSpeeds(-SLOW, -SLOW);
				}
			}
			else
			{
				double currentY = odometer.getY();
				while(Math.abs(currentY-odometer.getY()) < 5)
				{
					this.setSpeeds(-SLOW, -SLOW);
				}
			}
			this.setSpeeds(0,0);
			try{Thread.sleep(500);}catch(Exception e){}

			this.leftMotor.setSpeed(-SLOW);
			this.rightMotor.setSpeed(-SLOW);
			if(rightFirst)
			{
				this.leftMotor.rotate(-convertAngle(odometer.getWheelRadius(),odometer.getBaseWidth(),correctionAngle),true);
				this.rightMotor.rotate(convertAngle(odometer.getWheelRadius(),odometer.getBaseWidth(),correctionAngle),false);
			}
			else
			{
				this.leftMotor.rotate(convertAngle(odometer.getWheelRadius(),odometer.getBaseWidth(),correctionAngle),true);
				this.rightMotor.rotate(-convertAngle(odometer.getWheelRadius(),odometer.getBaseWidth(),correctionAngle),false);
			}	
		}
		this.setSpeeds(0, 0);
		correctHeading = false;
	}
	
	
	
	public void forwardTeam(){
		this.forward = true;
		this.defense = false;
	}
	
	public void defenseTeam(){
		this.defense = true;
		this.forward = false;
	}
	
	
	public void launchingLocalization(){
		this.collecting = true;
		double minLight = 0.3;
		
		boolean rightHit = false,leftHit = false;
		double firstHit = 0;
		double correction = 0;
		
		this.turnTo(0, true);
		
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
		this.setSpeeds(0,0);
		try{Thread.sleep(500);}catch(Exception e){}
		
		double currentX = odometer.getX();
		while(Math.abs(currentX-odometer.getX()) < LightLocalizerV4.lightSensorDistance)
		{
			this.setSpeeds(SLOW,SLOW);
		}
		
		this.setSpeeds(0,0);
		try{Thread.sleep(500);}catch(Exception e){}
		
		
		// Turn to appropriate y direction and travel until both sensors sense a black line and perform correction
		this.turnTo(90,true);
		
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
			
			double currentY = odometer.getY();
			while(Math.abs(currentY-odometer.getY()) < LightLocalizerV4.lightSensorDistance)
			{
				this.setSpeeds(SLOW,SLOW);
			}
			this.setSpeeds(0,0);
			this.collecting = false;
			try{Thread.sleep(500);}catch(Exception e){}
		
		
		this.collecting = false;
	}
	
	
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
		else{
			this.turnTo(0, true);
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
		this.setSpeeds(0,0);
		try{Thread.sleep(500);}catch(Exception e){}
		
		double currentX = odometer.getX();
		while(Math.abs(currentX-odometer.getX()) < LightLocalizerV4.lightSensorDistance)
		{
			this.setSpeeds(SLOW,SLOW);
		}
		
		this.setSpeeds(0,0);
		try{Thread.sleep(500);}catch(Exception e){}
		
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
			
			double currentY = odometer.getY();
			while(Math.abs(currentY-odometer.getY()) < LightLocalizerV4.lightSensorDistance)
			{
				this.setSpeeds(SLOW,SLOW);
			}
			this.setSpeeds(0,0);
			this.collecting = false;
			try{Thread.sleep(500);}catch(Exception e){}
	}
	
	
	
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
	
	
	
	private boolean isInDispenserZone()
	{
		if((odometer.getX() > Play.dispX-TILE_LENGTH && odometer.getX() < Play.dispX+TILE_LENGTH) && (odometer.getY() > Play.dispY-TILE_LENGTH && odometer.getY() < Play.dispY+TILE_LENGTH))
		{
			return true;
		}
		return false;
	}
	
	
	
	private boolean isDetectingBorder()
	{
		if(odometer.getX()<TILE_LENGTH/2 && (odometer.getAng() > 170 && odometer.getAng() < 190))
		{
			return true;
		}
		else if(odometer.getX()>boardDimensions*TILE_LENGTH-TILE_LENGTH/2 && (odometer.getAng() > 350 || odometer.getAng() < 10))
		{
			return true;
		}
		else if(odometer.getY()>boardDimensions*TILE_LENGTH-TILE_LENGTH/2 && (odometer.getAng() > 80 && odometer.getAng() < 100))
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
		avoiding = true;
		try{Thread.sleep(500);}catch(Exception e){}
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
		avoiding = false;
		try{Thread.sleep(500);}catch(Exception e){}
	}
	
	
	
	public float getColorDataR(){
		colorSensorR.fetchSample(colorDataR, 0);
		float lightStrength = colorDataR[0];
		return lightStrength;
	}
	
	public float getColorDataL(){
		colorSensorL.fetchSample(colorDataL, 0);
		float lightStrength = colorDataL[0];
		return lightStrength;
	}
	
	public boolean isCollecting(){
		return this.collecting;
	}

	
}