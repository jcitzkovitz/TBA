package tba;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/** The BaulLauncher class handles the catapult's motor's motion used to launch the ball.
 * */

public class BallLauncher {
	private EV3LargeRegulatedMotor leftCatapultMotor, rightCatapultMotor;
	private float distanceConstant;
	public BallLauncher (EV3LargeRegulatedMotor leftCat, EV3LargeRegulatedMotor rightCat, float distanceConstant){
		this.leftCatapultMotor= leftCat;
		this.rightCatapultMotor= rightCat;
		this.distanceConstant = distanceConstant;
	}
	public void launch(){
	//lift the ball

	this.leftCatapultMotor.setSpeed(20);
	this.rightCatapultMotor.setSpeed(20);
	this.leftCatapultMotor.rotate(-50,true);
	this.rightCatapultMotor.rotate(-50, false);
	Sound.beep();
	
	//launch the ball

	this.leftCatapultMotor.setAcceleration(2200);
	this.rightCatapultMotor.setAcceleration(2200);
<<<<<<< Updated upstream
	this.leftCatapultMotor.setSpeed(2500*this.distanceConstant);
	this.rightCatapultMotor.setSpeed(2500*this.distanceConstant);
	this.leftCatapultMotor.rotate(-100, true);
	this.rightCatapultMotor.rotate(-100,false);
=======
	this.leftCatapultMotor.setSpeed(2000);
	this.rightCatapultMotor.setSpeed(2000);
	this.leftCatapultMotor.rotate(-90, true);
	this.rightCatapultMotor.rotate(-90,false);
>>>>>>> Stashed changes
	Sound.twoBeeps();
	//return to the initial position
	this.leftCatapultMotor.setAcceleration(500);
	this.rightCatapultMotor.setAcceleration(500);
	this.leftCatapultMotor.setSpeed(90);
	this.rightCatapultMotor.setSpeed(90);
	this.leftCatapultMotor.rotate(140, true);
	this.rightCatapultMotor.rotate(140, false);
	Sound.twoBeeps();
	
	}
}
