package tba;

import lejos.robotics.SampleProvider;

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
		double correctXPoint = 0;
		double correctYPoint = 0;
		double currentAngle;
		boolean posX, posY, negX, negY;
		while(true&&!navi.isTurning())
		{
			if(getLightStrength() < minLight)
			{
				currentAngle = this.odo.getAng();
				
				posX = (currentAngle > 355 || currentAngle < 5);
				posY = (currentAngle > 85 && currentAngle < 95);
				negX = (currentAngle > 175 && currentAngle < 185);
				negY = (currentAngle > 265 && currentAngle < 275);
				
				if(posX)
				{
					if(!firstXLineCrossed)
					{
						correctXPoint = this.odo.getX();
						firstXLineCrossed=true;
					}
					else
					{
						correctXPoint+=30.48;
						this.odo.setPosition((new double[] {correctXPoint,0,0}), (new boolean[] {true,false,false}));
					}
				}
				else if(negX)
				{
					if(!firstXLineCrossed)
					{
						correctXPoint = this.odo.getX();
						firstXLineCrossed=true;
					}
					else
					{
						correctXPoint-=30.48;
						this.odo.setPosition((new double[] {correctXPoint,0,0}), (new boolean[] {true,false,false}));
					}
				}
				else if(posY)
				{
					if(!firstYLineCrossed)
					{
						correctYPoint = this.odo.getX();
						firstYLineCrossed=true;
					}
					else
					{
						correctYPoint+=30.48;
						this.odo.setPosition((new double[] {0,correctYPoint,0}), (new boolean[] {false,true,false}));
					}
				}
				else if(negY)
				{
					if(!firstYLineCrossed)
					{
						correctYPoint = this.odo.getX();
						firstYLineCrossed=true;
					}
					else
					{
						correctYPoint-=30.48;
						this.odo.setPosition((new double[] {0,correctYPoint,0}), (new boolean[] {false,true,false}));
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
}
