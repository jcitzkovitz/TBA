package tests;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class CatapultTest {

	private static final EV3LargeRegulatedMotor catMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
	
	public static void main(String[] args) {
		
		catMotor.setSpeed(catMotor.getMaxSpeed());
		catMotor.rotate(-130);
		
	}
	
}
