package tests;

import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class StraightTest {
	private Odometer odo;
	private Navigation navi;
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;

	public  StraightTest(Odometer odo, Navigation navi){
		this.odo= odo;
		this.navi=navi;
		EV3LargeRegulatedMotor[] motors = this.odo.getMotors();
		this.leftMotor = motors[0];
		this.rightMotor = motors[1];
	}
	public void goStraight(){
		while(odo.getY()<60.96){
			leftMotor.setSpeed(200);
			rightMotor.setSpeed(200);
			leftMotor.forward();
			rightMotor.forward();
			
		}
		Sound.beep();
		navi.turnTo(270, true);
		while(odo.getY()>=0){
			leftMotor.setSpeed(200);
			rightMotor.setSpeed(200);
			rightMotor.forward();
			leftMotor.forward();
			
		}
		navi.turnTo(90, true);
	}
	
}
