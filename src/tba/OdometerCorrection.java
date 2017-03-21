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
	private SampleProvider colorSensor;
	private float[] colorData;
	private boolean xDirection;
	private boolean yDirection;
	private double lastXLineCrossedPoint;
	private double lastYLineCrossedPoint;
	private double startingPosition;
	private boolean previousXPositive = true;
	private boolean previousYPositive = true;
	private double correctXPoint = 0;
	private double correctYPoint = 0;


	public OdometerCorrection(Odometer odo, Navigation navi,SampleProvider colorSensor, float[] colorData, double startingPosition)
	{
		this.odo = odo;
		this.navi = navi;
		this.colorSensor = colorSensor;
		this.colorData = colorData;
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
		while(true)
		{
			if(getLightStrength() < minLight&&!navi.isTurning())
			{
				Delay.msDelay(1000);
				currentAngle = this.odo.getAng();
				
				posX = (currentAngle > 355 || currentAngle < 5);
				posY = (currentAngle > 85 && currentAngle < 95);
				negX = (currentAngle > 175 && currentAngle < 185);
				negY = (currentAngle > 265 && currentAngle < 275);
				if(getLightStrength()<minLight){
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
			}
		}

	}

	private float getLightStrength(){
		colorSensor.fetchSample(colorData, 0);
		float lightStrength = colorData[0];
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
