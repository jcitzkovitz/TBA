package tests;

import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;


public class LightsensorTester {
	private Odometer odo;
	private SampleProvider colorSensor;
	private float[] colorData;	
	private Navigation navi;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;

	
	public LightsensorTester(Odometer odo, SampleProvider colorSensor, float[] colorData, Navigation navi){
		
	this.odo = odo;
	this.colorSensor = colorSensor;
	this.colorData = colorData;
	this.navi= navi;
	EV3LargeRegulatedMotor[] motors = this.odo.getMotors();
	this.leftMotor = motors[0];
	this.rightMotor = motors[1];
	}
	private float getLightStrength(){
		colorSensor.fetchSample(colorData, 0);
		float lightStrength = colorData[0];
		return lightStrength;
	}
	
	public void test(){
		
		int minLight = 13;
		/*if(getLightStrength()<minLight){
			Sound.beep();
		}*/
		
		navi.turnTo(0, true);
		while(odo.getX()<=60.96){
			leftMotor.setSpeed(200);
			rightMotor.setSpeed(205);
			leftMotor.forward();
			rightMotor.forward();
			if(getLightStrength()==minLight){
				Sound.beep();
			}
		}
		navi.turnTo(90, true);
		while(odo.getY()<=60.96){
			leftMotor.setSpeed(200);
			rightMotor.setSpeed(205);
			leftMotor.forward();
			rightMotor.forward();
			if(getLightStrength()==minLight){
				Sound.beep();
			}
		}
		navi.turnTo(180, true);
		while(odo.getX()>=0){
			leftMotor.setSpeed(200);
			rightMotor.setSpeed(205);
			leftMotor.forward();
			rightMotor.forward();
			if(getLightStrength()==minLight){
				Sound.beep();
			}
		}
		navi.turnTo(270, true);
		while(odo.getY()>=0){
			leftMotor.setSpeed(200);
			rightMotor.setSpeed(205);
			leftMotor.forward();
			rightMotor.forward();
			if(getLightStrength()==minLight){
				Sound.beep();
			}
		}
		navi.turnTo(0, true);
		
		
	}
	
	
	
	
}
