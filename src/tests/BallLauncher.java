package tests;

import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class BallLauncher {
	private EV3LargeRegulatedMotor leftCatapultMotor, rightCatapultMotor;
	
	public BallLauncher (EV3LargeRegulatedMotor leftCat, EV3LargeRegulatedMotor rightCat){
		this.leftCatapultMotor= leftCat;
		this.rightCatapultMotor= rightCat;
	}
	public void launch(){
	//launch the ball
	this.leftCatapultMotor.setAcceleration(2000);
	this.rightCatapultMotor.setAcceleration(2000);
	this.leftCatapultMotor.setSpeed(1500);
	this.rightCatapultMotor.setSpeed(1500);
	this.leftCatapultMotor.rotate(-150, true);
	this.rightCatapultMotor.rotate(-150,false);
	Sound.twoBeeps();
	//return to the initial position
	this.leftCatapultMotor.setAcceleration(500);
	this.rightCatapultMotor.setAcceleration(500);
	this.leftCatapultMotor.setSpeed(100);
	this.rightCatapultMotor.setSpeed(100);
	this.leftCatapultMotor.rotate(150, true);
	this.rightCatapultMotor.rotate(150, false);
	}
}
