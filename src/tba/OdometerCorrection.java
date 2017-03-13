package tba;

import lejos.robotics.SampleProvider;

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
		boolean blackLineCrossed = false;

		while(true)
		{
			if(getLightStrength() <= .25)
			{
				blackLineCrossed = true;
			}

			if(blackLineCrossed)
			{
				if(getLightStrength() - minLight >= .7)
				{

					/*Check which direction the robot is facing according to the odometer to decide whether
					to correct the x/y odometer readings.*/

					// +Y direction
					if(this.odo.getAng() >= 90-1 && this.odo.getAng() <= 90+1)
					{

						if(!firstYLineCrossed)
						{
							firstYLineCrossed = true;
							lastYLineCrossedPoint = this.odo.getY();
						}
						else
						{
							lastYLineCrossedPoint+=30.48;
							this.odo.setY(lastYLineCrossedPoint);
						}

					}

					// -Y direction
					else if (this.odo.getAng() >= 270-1 && this.odo.getAng() <= 270+1)
					{
						if(!firstYLineCrossed)
						{
							firstYLineCrossed = true;
							lastYLineCrossedPoint = this.odo.getY();
						}
						else
						{
							lastYLineCrossedPoint-=30.48;
							this.odo.setY(lastYLineCrossedPoint);
						}
					}

					// +X direction
					else if(this.odo.getAng() >= -1 && this.odo.getAng() <= 1)
					{
						if(!firstXLineCrossed)
						{
							firstXLineCrossed = true;
							lastXLineCrossedPoint = this.odo.getX();
						}
						else
						{
							lastXLineCrossedPoint+=30.48;
							this.odo.setX(lastYLineCrossedPoint);
						}
					}

					// -X direction
					else if(this.odo.getAng() >= 180-1 && this.odo.getAng() <= 180+1)
					{
						if(!firstXLineCrossed)
						{
							firstXLineCrossed = true;
							lastXLineCrossedPoint = this.odo.getX();
						}
						else
						{
							lastXLineCrossedPoint-=30.48;
							this.odo.setX(lastYLineCrossedPoint);
						}
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
