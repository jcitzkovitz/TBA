package tba;

import javax.management.timer.Timer;

import lejos.hardware.Sound;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

/**
 * The OdometerCorrection class corrects Odometer inaccuracies produced by wheel slippage
 * and other sources error while navigating*/

public class OdometerCorrection extends Thread {

	private Odometer odo;
	private Navigation navi;
	private SampleProvider colorSensorR;
	private float[] colorDataR;
	private SampleProvider colorSensorL;
	private float[] colorDataL;
	private boolean xDirection;
	private boolean yDirection;
	private double lastXLineCrossedPoint;
	private double lastYLineCrossedPoint;
	private double startingPosition;
	private boolean previousXPositive = true;
	private boolean previousYPositive = true;
	private double correctXPoint = 0;
	private double correctYPoint = 0;


	public OdometerCorrection(Odometer odo, Navigation navi,SampleProvider colorSensorR, float[] colorDataR,SampleProvider colorSensorL, float[] colorDataL, double startingPosition)
	{
		this.odo = odo;
		this.navi = navi;
		this.colorSensorR = colorSensorR;
		this.colorDataR = colorDataR;
		this.colorSensorL = colorSensorL;
		this.colorDataL = colorDataL;
		this.xDirection = false;
		this.yDirection = false;
		this.lastXLineCrossedPoint = 0;
		this.lastYLineCrossedPoint = 0;
		this.startingPosition = startingPosition;
	}

	public void run() {

		double minLight = 0.3;		//minimum light reflected by a black line
		boolean firstXLineCrossed = false;
		boolean firstYLineCrossed = false;
		double currentAngle;
		boolean posX, posY, negX, negY;
		int count = 0;
		while(true)
		{
			if((getLightStrengthL()<minLight||getLightStrengthR()<minLight)&&navi.isTurning())
				count++;
			
			if(count==2)
			{
				Delay.msDelay(1000);
				currentAngle = this.odo.getAng();
				
				posX = (currentAngle > 355 || currentAngle < 5);
				posY = (currentAngle > 85 && currentAngle < 95);
				negX = (currentAngle > 175 && currentAngle < 185);
				negY = (currentAngle > 265 && currentAngle < 275);
				if(getLightStrengthL()<minLight||getLightStrengthR()<minLight){
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
