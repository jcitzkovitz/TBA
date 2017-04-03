package tba;

import javax.management.timer.Timer;

import lejos.hardware.Sound;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

/**
 * @author William Wang
 * 
 * The OdometerCorrection class corrects Odometer inaccuracies produced by wheel slippage
 * and other sources error while navigating*/

public class OdometerCorrection extends Thread {

	private Odometer odo;
	private Navigation nav;
	private SampleProvider colorSensorR;
	private float[] colorDataR;
	private SampleProvider colorSensorL;
	private float[] colorDataL;
	private boolean xDirection;
	private boolean yDirection;
	private double lastXLineCrossedPoint;
	private double lastYLineCrossedPoint;
	private boolean previousXPositive = true;
	private boolean previousYPositive = true;
	private double correctXPoint = 0;
	private double correctYPoint = 0;


	/**
	 * OdometerCorrection constructor class
	 * @param odo Odometer object in charge of providing Odometer class methods
	 * @param nav Navigation object in charge of providing Navigation class methods
	 * @param colorSensorR Sample provider variable for the right color sensor
	 * @param colorSensorL Sample provider variable for the left color sensor
	 * @param colorDataR Float array to hold right color sensor data
	 * @param colorDataL Float array to hold left color sensor data
	 * */
	
	public OdometerCorrection(Odometer odo, Navigation nav,SampleProvider colorSensorR, float[] colorDataR,SampleProvider colorSensorL, float[] colorDataL)
	{
		this.odo = odo;
		this.nav = nav;
		this.colorSensorR = colorSensorR;
		this.colorDataR = colorDataR;
		this.colorSensorL = colorSensorL;
		this.colorDataL = colorDataL;
		this.xDirection = false;
		this.yDirection = false;
		this.lastXLineCrossedPoint = 0;
		this.lastYLineCrossedPoint = 0;
	}

	/**
	 * Runs the odometerCorrection thread. This method uses a light sensor to correct the distance traveled
	 * by the robot. After each detected line while traveling in a specific direction, the robot's odometer
	 * valuse should be corrected to what the reading was at the previous detected line PLUS the length of a
	 * tile (30.48 cm).
	 * */
	public void run() {

		double minLight = 0.3;		//minimum light reflected by a black line
		boolean firstXLineCrossed = false;
		boolean firstYLineCrossed = false;
		double currentAngle;
		boolean posX, posY, negX, negY;
		int count = 0;
		while(true)
		{
			if((getColorDataL()<minLight||getColorDataR()<minLight)&&nav.isTurning())
				count++;
			
			if(count==2)
			{
				Delay.msDelay(1000);
				currentAngle = this.odo.getAng();
				
				posX = (currentAngle > 355 || currentAngle < 5);
				posY = (currentAngle > 85 && currentAngle < 95);
				negX = (currentAngle > 175 && currentAngle < 185);
				negY = (currentAngle > 265 && currentAngle < 275);
				if(getColorDataL()<minLight||getColorDataR()<minLight){
					posX = false;
					posY = false;
					negX = false;
					negY = false;
				}
				if(posX)
				{
					if(!firstXLineCrossed)
					{
						correctXPoint = this.odo.getX();
						firstXLineCrossed=true;
						previousXPositive = true;
					}
					else
					{
						if(previousXPositive){
							correctXPoint+=30.48;
						}
						setPosition(correctXPoint, 0);
						previousXPositive = true;
					}
				}
				else if(negX)
				{
					if(!firstXLineCrossed)
					{
						correctXPoint = this.odo.getX();
						firstXLineCrossed=true;
						previousXPositive = false;
					}
					else
					{
						if(!previousXPositive){
							correctXPoint-=30.48;
						}
						setPosition(correctXPoint, 0);
						previousXPositive = false;
					}
				}
				else if(posY)
				{
					if(!firstYLineCrossed)
					{
						correctYPoint = this.odo.getX();
						firstYLineCrossed=true;
						previousYPositive = true;
					}
					else
					{
						if(previousYPositive){
							correctYPoint+=30.48;
						}
						setPosition(correctYPoint, 1);
						previousYPositive = true;
					}
				}
				else if(negY)
				{
					if(!firstYLineCrossed)
					{
						correctYPoint = this.odo.getX();
						firstYLineCrossed=true;
						previousYPositive = false;
					}
					else
					{
						if(!previousYPositive){
							correctYPoint-=30.48;
						}
						setPosition(correctYPoint, 1);
						previousYPositive = false;
					}
				}
				
				count=0;
			}
		}

	}

	/**
	 * Get the the light strength from the right color sensor
	 * 
	 * @return Right color sensor value
	 * */
	private float getColorDataR(){
		colorSensorR.fetchSample(colorDataR, 0);
		float lightStrength = colorDataR[0];
		return lightStrength;
	}
	
	/**
	 * Get the the light strength from the left color sensor
	 * 
	 * @return Left color sensor value
	 * */
	private float getColorDataL(){
		colorSensorL.fetchSample(colorDataL, 0);
		float lightStrength = colorDataL[0];
		return lightStrength;
	}
	
	/**
	 * Set the x and y values in the odometer accordingly
	 * 
	 * @param value New value to set
	 * @param XorY States whether the correction must be done in the x or y values
	 * */
	private void setPosition(double value, int XorY){
		if(XorY==0){
			if(Math.abs(value-odo.getX())<5){
				this.odo.setX(value);
			}
			else if(Math.abs(value-odo.getX())>25){
				correctXPoint = odo.getX();
			}
		}
		if(XorY==1){
			if(Math.abs(value-odo.getY())<5){
				this.odo.setY(value);
			}
			else if(Math.abs(value-odo.getX())>25){
				correctXPoint = odo.getY();
			}
		}
	}
}
