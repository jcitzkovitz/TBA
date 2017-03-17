package tba;

import lejos.robotics.SampleProvider;

/**
 * The OdometerCorrection class corrects Odometer inaccuracies produced by wheel slippage
 * and other sources error while navigating*/

public class OdometerCorrection extends Thread {

	private Odometer odo;
	private SampleProvider colorSensor;
	private float[] colorData;
	private boolean xDirection;
	private boolean yDirection;
	private double lastXLineCrossedPoint;
	private double lastYLineCrossedPoint;
	private double startingPosition;

	public OdometerCorrection(Odometer odo,SampleProvider colorSensor, float[] colorData, double startingPosition)
	{
		this.odo = odo;
		this.colorSensor = colorSensor;
		this.colorData = colorData;
		this.xDirection = false;
		this.yDirection = false;
		this.lastXLineCrossedPoint = 0;
		this.lastYLineCrossedPoint = 0;
		this.startingPosition = startingPosition;
	}

	public void run() {

		double minLight = 0.25;		//minimum light reflected by a black line
		boolean firstXLineCrossed = false;
		boolean firstYLineCrossed = false;
		double correctXPoint = 0;
		double correctYPoint = 0;
		double currentAngle;
		boolean posX, posY, negX, negY;
		while(true)
		{
			if(getLightStrength() < minLight)
			{
				currentAngle = this.odo.getAng();
				
				posX = (currentAngle > -1 && currentAngle < 1);
				posY = (currentAngle > 89 && currentAngle < 91);
				negX = (currentAngle > 179 && currentAngle < 181);
				negY = (currentAngle > 269 && currentAngle < 271);
				
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
