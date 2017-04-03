package tba;

import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

/**
 * The LightLocalizerV2 class is the secon version od LightLocalizer. This class performs localization through the use of a light sensor in terms of the robot's
 * x and y position on the grid. It tracks the distance of both the x and y axis lines from its original 
 * position by driving forward until a line is detected for both directions. The robot then advances to its 
 * starting position where it's correct coordinates are set.*/
 

public class LightLocalizerV2 {
	private Odometer odo;
	private SampleProvider colorSensor;
	private float[] colorData;	
	private Navigation navi;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	private double lightDistance = 15.8;
	private int corner;
	private final int SPEED = 100;
	private float color;
	
	public LightLocalizerV2(Odometer odo, SampleProvider colorSensor, float[] colorData, Navigation navi, int startingCorner) {
		this.odo = odo;
		this.colorSensor = colorSensor;
		this.colorData = colorData;
		this.navi= navi;
		EV3LargeRegulatedMotor[] motors = this.odo.getMotors();
		this.leftMotor = motors[0];
		this.rightMotor = motors[1];
		this.corner = startingCorner;
	}
	
	public void doLocalization() {
		
//		double minLight =0.3;
//		navi.turnTo(0, true);
//		while(getColorData()<minLight){
//			this.leftMotor.setSpeed(150);
//			this.leftMotor.setSpeed(150);
//			this.leftMotor.forward();
//			this.rightMotor.forward();
//		}
//		odo.setPosition(new double[]{0, odo.getY(), odo.getAng()}, new boolean[] {true, true, true});
//		this.navi.drive(-5);
//		navi.turnTo(90, true);
//		while(getColorData()<minLight){
//			this.leftMotor.setSpeed(150);
//			this.leftMotor.setSpeed(150);
//			this.leftMotor.forward();
//			this.rightMotor.forward();
//		}
//		odo.setPosition(new double[]{odo.getX(), 0, odo.getAng()}, new boolean[] {true, true, true});
//		
//		if(corner==2){
//			this.odo.setPosition(new double[]{304.8-odo.getY(),odo.getX(),90+odo.getAng()}, new boolean [] {true, true, true});
//		}
//		if(corner==3){
//			this.odo.setPosition(new double[]{304.8-odo.getX(),304.8-odo.getX(),180+odo.getAng()}, new boolean [] {true, true, true});
//		}
//		if(corner==4){
//			if((odo.getAng()+270)<360){
//				this.odo.setPosition(new double[]{odo.getY(),304.8-odo.getX(),270+odo.getAng()}, new boolean [] {true, true, true});
//			}
//			else{
//				this.odo.setPosition(new double[]{odo.getY(),304.8-odo.getX(),odo.getAng()-90}, new boolean [] {true, true, true});
//			}
//		}
//		
		/*
		// Move the robot forward by a 30 degree rotation
		this.leftMotor.setSpeed(SPEED);
		this.rightMotor.setSpeed(SPEED);
		
		this.leftMotor.rotate(30,true);
		this.rightMotor.rotate(30,false);
		
		navi.setSpeeds(0, 0);
		
		// Rotate the robot to an angle of 45 degrees
		navi.turnTo(45, true);
		
		navi.setSpeeds(0, 0);
		
		try{Thread.sleep(5000);}catch(Exception e){}
		
		// *** Begin light localization ***
		
		int lineCounter = 0;
		double[] lineAngles = new double[4];
		double maxLight = 40;
		double currentAngle = odo.getAng();
		while(lineCounter < 4)
		{
			navi.setSpeeds(-SPEED, SPEED);
			//initiate the sensor to retrieve the values 
			colorSensor.fetchSample(colorData, 0);
			 color = (colorData[0]*100);
			 
			 if (color < maxLight){
				 //if a line is intersected, play sound and record the value in the array
				 Sound.playNote(Sound.FLUTE, 440, 250);
				 lineAngles[lineCounter] = odo.getAng();
				 lineCounter++;
			 }
		}

		
		double lightSensorDistanceToCenterOfRot = 8;
		
		double angleX = (lineAngles[3] - lineAngles[1])/2;
		double angleY = (lineAngles[2] - lineAngles[0])/2;
		
		double lengthX = -1*lightSensorDistanceToCenterOfRot*Math.cos(angleY*Math.PI/180);
		double lengthY = -1*lightSensorDistanceToCenterOfRot*Math.cos(angleX*Math.PI/180);
		
		double deltaTheta1 = 90 + angleY/2 - (lineAngles[0]-180);
		double deltaTheta2 = 90 + angleX/2 - (lineAngles[1]-180);
		double deltaTheta = (deltaTheta1+deltaTheta2)/2;
		double theta = Math.abs(odo.getAng()-deltaTheta);
		
		this.odo.setPosition(new double [] {(lengthX), (lengthY), theta}, new boolean [] {true, true, true});
		Sound.beep();
		try{Thread.sleep(3000);}catch(Exception e){}
		
		navi.turnTo(90, true);
		try{Thread.sleep(5000);}catch(Exception e){}
		
		navi.travelTo(0, 0);
		try{Thread.sleep(5000);}catch(Exception e){}
		
		navi.turnTo(0, true);
		try{Thread.sleep(5000);}catch(Exception e){}
		*/
		
	}
	
	public float getColorData(){
		colorSensor.fetchSample(colorData, 0);
		float lightStrength = colorData[0];
		return lightStrength;
	}
}
