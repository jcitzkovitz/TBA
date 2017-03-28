package tba;

import lejos.hardware.Sound;
import lejos.robotics.SampleProvider;

/**
 * This is the second version of Odmetry correction
 * The OdometerCorrection class corrects Odometer inaccuracies produced by wheel slippage
 * and other sources error while navigating*/

public class OdometerCorrectionV2 extends Thread {
	private static final long CORRECTION_PERIOD = 10;
	private Odometer odo;
	private SampleProvider colorSensorR;
	private float[] colorDataR;
	private SampleProvider colorSensorL;
	private float[] colorDataL;
	private double minLight = 0.3;
	private double lightSensorDistance = 3;
	private double lightSensorAngle = -10;
	private static boolean doCorrection;
	
	public OdometerCorrectionV2(Odometer odo, SampleProvider colorSensorR, float[] colorDataR, SampleProvider colorSensorL, float[] colorDataL){
		this.odo = odo;
		this.colorSensorR = colorSensorR;
		this.colorDataR = colorDataR;
		this.colorSensorL = colorSensorL;
		this.colorDataL = colorDataL;
	}
	
	public void run() {
		long correctionStart, correctionEnd;
		boolean rightHit = false, leftHit = false, rightFirst = false, leftFirst = false;
		double initialXPoint = 0, initialYPoint = 0;
		
		while (true) {
			correctionStart = System.currentTimeMillis();
			if(!Navigation.isTurning())
			{
				if(getLightStrengthR()<minLight)
				{
					Sound.beep();
					rightHit=true;
					if(!leftHit)
					{
						rightFirst = true;
						initialXPoint = this.odo.getX();
						initialYPoint = this.odo.getY();
					}
				}
				if(getLightStrengthL()<minLight)
				{
					Sound.beep();
					Sound.beep();
					leftHit=true;
					if(!rightHit)
					{
						leftFirst = true;
						initialXPoint = this.odo.getX();
						initialYPoint = this.odo.getY();
					}
				}

				if(leftHit && rightHit)
				{
					Sound.beep();
					Sound.beep();
					Sound.beep();
					double xDist = Math.abs(initialXPoint-this.odo.getX());
					double yDist = Math.abs(initialYPoint-this.odo.getY());
					double xAng = Math.asin(xDist/odo.getBaseWidth());
					double yAng = Math.asin(yDist/odo.getBaseWidth());
					if(rightFirst)
					{
						
					}
					else
					{	
						
					}

					leftHit=false;
					leftFirst=false;
					rightHit=false;
					rightFirst=false;
				}
				if(doCorrection){
					Sound.beep();
					Sound.beep();
					Sound.beep();
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
					doCorrection = false;
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
	
	public static void doCorrection()
	{
		doCorrection = true;
	}
	
}
