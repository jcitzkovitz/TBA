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
<<<<<<< Updated upstream
	this.leftCatapultMotor.setAcceleration(2000);
	this.rightCatapultMotor.setAcceleration(2000);
	this.leftCatapultMotor.setSpeed(1500);
	this.rightCatapultMotor.setSpeed(1500);
	this.leftCatapultMotor.rotate(-120, true);
	this.rightCatapultMotor.rotate(-120,false);
	Sound.twoBeeps();
	//return to the initial position
	this.leftCatapultMotor.setAcceleration(500);
	this.rightCatapultMotor.setAcceleration(500);
	this.leftCatapultMotor.setSpeed(100);
	this.rightCatapultMotor.setSpeed(100);
	this.leftCatapultMotor.rotate(120, true);
	this.rightCatapultMotor.rotate(120, false);
=======
	this.leftCatapultMotor.setAcceleration(3000);
	this.rightCatapultMotor.setAcceleration(3000);
	this.leftCatapultMotor.setSpeed(2500);
	this.rightCatapultMotor.setSpeed(2500);
	this.leftCatapultMotor.rotate(85, true);
	this.rightCatapultMotor.rotate(85,false);
	Sound.twoBeeps();
	//return to the initial position
	this.leftCatapultMotor.setAcceleration(100);
	this.rightCatapultMotor.setAcceleration(100);
	this.leftCatapultMotor.setSpeed(90);
	this.rightCatapultMotor.setSpeed(90);
	this.leftCatapultMotor.rotate(-85, true);
	this.rightCatapultMotor.rotate(-85, false);
>>>>>>> Stashed changes
	}
}
