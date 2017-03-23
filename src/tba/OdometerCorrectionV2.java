package tba;

import lejos.hardware.Sound;
import lejos.robotics.SampleProvider;

/**
 * This is the second version of Odmetry correction
 * The OdometerCorrection class corrects Odometer inaccuracies produced by wheel slippage
 * and other sources error while navigating*/

public class OdometerCorrectionV2 {
	private static final long CORRECTION_PERIOD = 10;
	private Odometer odo;
	private SampleProvider colorSensor;
	private float[] colorData;
	private double minLight = 0.3;
	private double lightSensorDistance = 3;
	private double lightSensorAngle = -10;
	
	public OdometerCorrectionV2(Odometer odo, SampleProvider colorSensor, float[] colorData){
		this.odo = odo;
		this.colorSensor = colorSensor;
		this.colorData = colorData;
	}
	
	public void run() {
		long correctionStart, correctionEnd;
		while (true) {
			correctionStart = System.currentTimeMillis();

			if(getLightStrength()<minLight){
				Sound.beep();
				
				double theta=this.odo.getPosition()[2];
				double sensorX=this.odo.getX()+lightSensorDistance*Math.cos(Math.toRadians(theta+lightSensorAngle));
				double sensorY=this.odo.getY()+lightSensorDistance*Math.sin(Math.toRadians(theta+lightSensorAngle));
				//check the robot is moving in x direction or y direction,if the theta is about 90 or 270 degree, it is traveling in x direction
				double deltaX = sensorX%30.48;
				if(deltaX>30.48/2){
					deltaX-=30.48;
				}
				double deltaY = sensorY%30.48;
				if(deltaY>30.48/2){
					deltaY-=30.48;
				}
				if(Math.abs(deltaX)<Math.abs(deltaY)){
					double x = odo.getX()-deltaX;
					odo.setX(x);
				}
				else{
					double y = odo.getY()-deltaY;
					odo.setY(y);
				}
			 }
			 
			// this ensure the odometry correction occurs only once every period
			correctionEnd = System.currentTimeMillis();
			if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
				try {
					Thread.sleep(CORRECTION_PERIOD
							- (correctionEnd - correctionStart));
				} catch (InterruptedException e) {
					// there is nothing to be done here because it is not
					// expected that the odometry correction will be
					// interrupted by another thread
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
