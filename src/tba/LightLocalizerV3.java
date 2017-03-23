package tba;

import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

/**
 * The LightLocalizerV2 class is the secon version od LightLocalizer. This class performs localization through the use of a light sensor in terms of the robot's
 * x and y position on the grid. It tracks the distance of both the x and y axis lines from its original 
 * position by driving forward until a line is detected for both directions. The robot then advances to its 
 * starting position where it's correct coordinates are set.*/
 

public class LightLocalizerV3 {
	private Odometer odo;
	private SampleProvider colorSensor;
	private float[] colorData;	
	private Navigation navi;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	private float SPEED = 100;
	private double lightSensorDistance = 3;
	
	public LightLocalizerV3(Odometer odo, SampleProvider colorSensor, float[] colorData, Navigation navi) {
		this.odo = odo;
		this.colorSensor = colorSensor;
		this.colorData = colorData;
		this.navi= navi;
		EV3LargeRegulatedMotor[] motors = this.odo.getMotors();
		this.leftMotor = motors[0];
		this.rightMotor = motors[1];
	}
	
	public void doLocalization() {
		
		double minLight =0.3;
		
		/*Travel in the x direction until a black line is detected, and track the
		 * distance traveled*/
		
		double initialXPosition = this.odo.getX();
		double traveledXPosition;
		
		// Travel in the x direction until a black line is seen
		while(true)
		{
			this.navi.setSpeeds(SPEED,SPEED);
			if(getColorData() < minLight)
			{
				Sound.beep();
				traveledXPosition = this.odo.getX();
				break;
			}
		}
		navi.setSpeeds(0, 0);
		try{Thread.sleep(1000);}catch(Exception e){}
		/*Travel traveledXPosition-initialXPosition backwards, turn 90 degrees and
		 * travel in the y direction until a black line is detected and track that
		 * distance traveled*/
		
		double traveledXDistance = traveledXPosition-initialXPosition;
		
		// Travel backwards until the robot reaches the initial x position
		while(this.odo.getX() > initialXPosition)
		{
			this.navi.setSpeeds(-SPEED,-SPEED);
		}

		navi.setSpeeds(0, 0);
		try{Thread.sleep(1000);}catch(Exception e){}
		
		// Turn to 90 degrees
		this.navi.turnTo(90,true);
		
		// Travel in the y direction until a black line is seen
		while(true)
		{
			this.navi.setSpeeds(SPEED,SPEED);
			if(getColorData() < minLight)
			{
				Sound.beep();
				break;
			}
		}
		
		navi.setSpeeds(0, 0);
		try{Thread.sleep(1000);}catch(Exception e){}
		
		/* Travel the lightSensorDistance in the y direction, turn to 0 degrees,
		 * and travel traveledXDistance+lightSensorDistance in the x direction. 
		 * After this step, localization will be achieved.*/
		double currentYPosition = this.odo.getY();
		
		// Travel lightSensorDistance in the y direction
		while(this.odo.getY() < currentYPosition+lightSensorDistance)
		{
			this.navi.setSpeeds(SPEED,SPEED);
		}
		Sound.beep();
		
		navi.setSpeeds(0, 0);
		try{Thread.sleep(1000);}catch(Exception e){}
		
		// Turn to 0 degrees
		this.navi.turnTo(0,true);
		
		navi.setSpeeds(0, 0);
		try{Thread.sleep(1000);}catch(Exception e){}
		
		// Travel traveledXDistance+lightSensorDistance in the x direction
		while(this.odo.getX() < traveledXDistance+lightSensorDistance)
		{
			this.navi.setSpeeds(SPEED,SPEED);
		}
		
		navi.setSpeeds(0, 0);
		try{Thread.sleep(1000);}catch(Exception e){}
		
		// Set the odometer fields to (0,0,0)
		this.odo.setPosition((new double[] {0,0,0}), (new boolean[] {true,true,true}));
		
		Sound.beep();
		
		try{Thread.sleep(1000);}catch(Exception e){};
		
	}
	
	public float getColorData(){
		colorSensor.fetchSample(colorData, 0);
		float lightStrength = colorData[0];
		return lightStrength;
	}
}
