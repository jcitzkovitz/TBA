package tba;

import lejos.hardware.Sound;
import lejos.robotics.SampleProvider;

/** The CorrectHeading class corrects the heading of the robot through the use of two light sensors
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
	
	public CorrectHeading(Odometer odo, Navigation nav, SampleProvider colorSensorR, float[] colorDataR, SampleProvider colorSensorL, float[] colorDataL)
	{
		this.odo = odo;
		this.nav = nav;
		this.colorSensorR = colorSensorR;
		this.colorDataR = colorDataR;
		this.colorSensorL = colorSensorL;
		this.colorDataL = colorDataL;
	}
	
	public void run() {
		
		boolean rightHit = false,leftHit = false;
		double firstHit = 0;
		double correction = 0;
		int direction = 0;
		int countR = 0;
		int countL = 0;
		while(true)
		{
			if(!nav.isTurning())
			{
				
				if(odo.getAng()<5||odo.getAng()>355||(odo.getAng()<185&&odo.getAng()>175)){
					direction = 0;
				}
				else if((odo.getAng()<95&&odo.getAng()>85)||(odo.getAng()<275&&odo.getAng()>265)){
					direction = 1;
				}
				if(getLightStrengthR()<minLight)
				{
					countR++;
					if(leftHit)
					{
						if(countR < 5)
						{
							if(direction == 0){
								correction = this.odo.getX()-firstHit+.5;
							}
							else if(direction == 1){
								correction = this.odo.getY()-firstHit+.5;
							}

							correction = Math.toDegrees(Math.asin(correction/odo.getBaseWidth()));
							Sound.beep();
							nav.correctHeading(false, correction);
							leftHit=false;
							rightHit=false;
						}
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

				if(getLightStrengthL()<minLight)
				{
					countL++;
					if(rightHit)
					{
						if(countL < 5)
						{
							if(direction == 0){
								correction = this.odo.getX()-firstHit+.5;
							}
							else if(direction == 1){
								correction = this.odo.getY()-firstHit+.5;
							}

							correction = Math.toDegrees(Math.asin(correction/odo.getBaseWidth()));

							Sound.buzz();
							nav.correctHeading(true, correction);
							leftHit=false;
							rightHit=false;
						}
						countL = 0;
						countR = 0;
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

			}
		}
		
	}
	
	private float getLightStrengthR(){
		colorSensorR.fetchSample(colorDataR, 0);
		float lightStrength = colorDataR[0];
		return lightStrength;
	}
	
	private float getLightStrengthL(){
		colorSensorL.fetchSample(colorDataL, 0);
		float lightStrength = colorDataL[0];
		return lightStrength;
	}
}
