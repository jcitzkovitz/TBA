package tba;

import lejos.hardware.Sound;
import lejos.robotics.SampleProvider;

/** 
 * @author Jordan Itzkovitz and William Wang
 * 
 * The CorrectHeading class corrects the heading of the robot through the use of two light sensors
 * located in front of the robot - the left sensor by the left wheel and the right sensor by the right
 * wheel. This correction is done based on which sensor senses a black line first, and performs
 * calculations to rotate the robot in the proper direction to fix it's heading.
 * */

public class CorrectHeading extends Thread{

	private Odometer odo;
	private Navigation nav;
	private SampleProvider colorSensorR;
	private float[] colorDataR;
	private SampleProvider colorSensorL;
	private float[] colorDataL;
	private double minLight = 0.3;
	
	/**
	 * CorrectHeading constructor class
	 * @param odo Odometer object in charge of providing Odometer class methods
	 * @param nav Navigation object in charge of providing Navigation class methods
	 * @param colorSensorR Sample provider variable for the right color sensor
	 * @param colorSensorL Sample provider variable for the left color sensor
	 * @param colorDataR Float array to hold right color sensor data
	 * @param colorDataL Float array to hold left color sensor data
	 * */
	public CorrectHeading(Odometer odo, Navigation nav, SampleProvider colorSensorR, float[] colorDataR, SampleProvider colorSensorL, float[] colorDataL)
	{
		this.odo = odo;
		this.nav = nav;
		this.colorSensorR = colorSensorR;
		this.colorDataR = colorDataR;
		this.colorSensorL = colorSensorL;
		this.colorDataL = colorDataL;
	}
	
	/**
	 * Method which runs CorrectHeading thread
	 * */
	public void run() {
		
		//Booleans which tells the program which sensor hit the black line first
		boolean rightHit = false,leftHit = false;
		
		//Hold the first hit value
		double firstHit = 0;
		
		//Overall correction needed
		double correction = 0;
		
		//Which direction the robot is traveling in
		int direction = 0;
		
		//Counts that will help notify the program if one of the light sensors are traveling along a black
		//line, and to disregard the correction if so
		int countR = 0;
		int countL = 0;
		
		while(true)
		{
			//If the robot is not turning or collectin the ball, check if correction is needed
			if((!nav.isTurning())&&(!nav.isCollecting()))
			{
				//If the robot is traveling in the x direction, set direction to 0, else set it to 1
				if(odo.getAng()<10||odo.getAng()>350||(odo.getAng()<190&&odo.getAng()>170)){
					direction = 0;
				}
				else if((odo.getAng()<100&&odo.getAng()>80)||(odo.getAng()<280&&odo.getAng()>260)){
					direction = 1;
				}
				
				/*
				 * The following if statements read the color sensor data and test it against the minLight
				 * variable. It uses the odometer readings to calculate what heading correction is needed
				 * based on when both the right and left sensors hit a black line. For example, if travelling
				 * forward and the left sensor hits a line prior to the right, the robot will turn counter
				 * clockwise to correct its heading. After the motion is finished, the angle is reset in
				 * the odometer. The counters filter out the readings when either of the sensors are travelling
				 * along a black line.
				 * */
				
				if(getLightStrengthR()<minLight)
				{
					countR++;
					if(countR < 8)
					{
						if(leftHit)
						{
							if(direction == 0){
								correction = Math.abs(this.odo.getX()-firstHit)+.5;
							}
							else if(direction == 1){
								correction = Math.abs(this.odo.getY()-firstHit)+.5;
							}

							correction = Math.toDegrees(Math.asin(correction/odo.getBaseWidth()));
							Sound.beep();
							nav.correctHeading(false, correction);
							leftHit=false;
							rightHit=false;
							countR = 0;
							countL = 0;
						}
						else{
							rightHit = true;
							if(direction == 0){
								firstHit = this.odo.getX();
							}
							else if(direction == 1){
								firstHit = this.odo.getY();
							}

						}
					}
					else
					{
						countR = 0;
						countL = 0;
					}
				}

				if(getLightStrengthL()<minLight)
				{
					countL++;
					if(countL < 8)
					{
						if(rightHit)
						{
							if(direction == 0){
								correction = Math.abs(this.odo.getX()-firstHit)+.5;
							}
							else if(direction == 1){
								correction = Math.abs(this.odo.getY()-firstHit)+.5;
							}

							correction = Math.toDegrees(Math.asin(correction/odo.getBaseWidth()));

							Sound.buzz();
							nav.correctHeading(true, correction);
							leftHit=false;
							rightHit=false;
							countR = 0;
							countL = 0;
						}
						else{
							leftHit = true;
							if(direction == 0){
								firstHit = this.odo.getX();
							}
							else if(direction == 1){
								firstHit = this.odo.getY();
							}

						}
					}
					else
					{
						countR = 0;
						countL = 0;
					}
				}

			}
		}
		
	}
	
	/**
	 * Get the the light strength from the right color sensor
	 * 
	 * */
	private float getLightStrengthR(){
		colorSensorR.fetchSample(colorDataR, 0);
		float lightStrength = colorDataR[0];
		return lightStrength;
	}
	
	/**
	 * Get the the light strength from the left color sensor
	 * 
	 * */
	private float getLightStrengthL(){
		colorSensorL.fetchSample(colorDataL, 0);
		float lightStrength = colorDataL[0];
		return lightStrength;
	}
}
