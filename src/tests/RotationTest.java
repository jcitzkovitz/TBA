package tests;

import tba.Navigation;
import tba.Odometer;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class RotationTest {
	private Odometer odo;
	private Navigation navi;
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;

	public  RotationTest(Odometer odo, Navigation navi){
		this.odo= odo;
		this.navi=navi;
		EV3LargeRegulatedMotor[] motors = this.odo.getMotors();
		this.leftMotor = motors[0];
		this.rightMotor = motors[1];
	}
	
	public void rotate(double angle){
		
		Sound.beep();
		navi.turnTo(angle,true);
		Sound.beep();
	}
	
}