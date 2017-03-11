package tests;

import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class BallLauncher {
	private EV3LargeRegulatedMotor liftMotor, launchMotor;
	
	public BallLauncher (EV3LargeRegulatedMotor lift, EV3LargeRegulatedMotor launch){
		this.liftMotor= lift;
		this.launchMotor= launch;
	}
	public void launch(){
	
		this.liftMotor.setAcceleration(200);
		this.liftMotor.setSpeed(100);
		this.liftMotor.rotate(-100, false);
		Sound.beep();
		this.liftMotor.setSpeed(100);
		this.liftMotor.rotate(100,false);
		
		this.launchMotor.setAcceleration(9000);
		this.launchMotor.setSpeed(9000);
		this.launchMotor.rotate(130, false);
		Sound.beep();
		this.launchMotor.setSpeed(100);
		this.launchMotor.rotate(-130, false);
	
	}

}
