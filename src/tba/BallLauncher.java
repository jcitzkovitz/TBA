package tba;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/** 
 * @author	Bijan Sharihari
 * @author	Thomas Phillipon
 * 
 * The BaulLauncher class handles the catapult's motor's motion used to launch the ball.
 * 
 * @param	leftCatapultMotor	The left catapult motor variable
 * @param 	rightCatapultMotor	The right catapult motor variable
 * 
 * */

public class BallLauncher {
	
	private EV3LargeRegulatedMotor leftCatapultMotor, rightCatapultMotor;
	private float distanceConstant;		//Launching ratio based on distance
	
	public BallLauncher (EV3LargeRegulatedMotor leftCat, EV3LargeRegulatedMotor rightCat, float distanceConstant){
		this.leftCatapultMotor= leftCat;
		this.rightCatapultMotor= rightCat;
		this.distanceConstant = distanceConstant;
	}
	
	/**
	 * launch() controls the motors in a specific manner to launch the ball as a catapult 
	 * 
	 * @return		void
	 * */
	public void launch(){
	
	//lift the ball
	this.leftCatapultMotor.setSpeed(20);
	this.rightCatapultMotor.setSpeed(20);
	this.leftCatapultMotor.rotate(-45,true);
	this.rightCatapultMotor.rotate(-45, false);
	Sound.beep();
	
	//launch the ball
	this.leftCatapultMotor.setAcceleration(3300);
	this.rightCatapultMotor.setAcceleration(3300);

	/*this.leftCatapultMotor.setSpeed(2500*this.distanceConstant);
	this.rightCatapultMotor.setSpeed(2500*this.distanceConstant);
	this.leftCatapultMotor.rotate(-100, true);
	this.rightCatapultMotor.rotate(-100,false);*/
	
	this.leftCatapultMotor.setSpeed(3000*distanceConstant);
	this.rightCatapultMotor.setSpeed(3000);
	this.leftCatapultMotor.rotate(-95, true);
	this.rightCatapultMotor.rotate(-95,false);
	 
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
