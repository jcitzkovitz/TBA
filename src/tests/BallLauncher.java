package tests;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class BallLauncher {
	private EV3LargeRegulatedMotor leftCatapultMotor, rightCatapultMotor;
	
	public BallLauncher (EV3LargeRegulatedMotor leftCat, EV3LargeRegulatedMotor rightCat){
		this.leftCatapultMotor= leftCat;
		this.rightCatapultMotor= rightCat;
	}
	public void launch(){
	//lift the ball

	this.leftCatapultMotor.setAcceleration(10);
	this.rightCatapultMotor.setAcceleration(10);
	this.leftCatapultMotor.setSpeed(15);
	this.rightCatapultMotor.setSpeed(15);
	this.leftCatapultMotor.rotate(-40,true);
	this.rightCatapultMotor.rotate(-40, false);
	Sound.beep();
	Button.waitForAnyPress();
	//launch the ball

	this.leftCatapultMotor.setAcceleration(2000);
	this.rightCatapultMotor.setAcceleration(2000);
	this.leftCatapultMotor.setSpeed(2000);
	this.rightCatapultMotor.setSpeed(2000);
	this.leftCatapultMotor.rotate(-110, true);
	this.rightCatapultMotor.rotate(-110,false);
	Sound.twoBeeps();
	//return to the initial position
	this.leftCatapultMotor.setAcceleration(500);
	this.rightCatapultMotor.setAcceleration(500);
	this.leftCatapultMotor.setSpeed(90);
	this.rightCatapultMotor.setSpeed(90);
	this.leftCatapultMotor.rotate(150, true);
	this.rightCatapultMotor.rotate(150, false);
	Sound.twoBeeps();
	
	}
}
