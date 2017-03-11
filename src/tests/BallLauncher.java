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
	
		/*this.liftMotor.setAcceleration(10000);
		this.liftMotor.setSpeed(10000);
		this.liftMotor.rotate(-90, false);
		Sound.beep();
		this.liftMotor.setSpeed(100);
		this.liftMotor.rotate(90,false);*/
		this.launchMotor.setAcceleration(2200);
		this.launchMotor.setSpeed(1200);
		this.launchMotor.rotate(-150, false);
		this.launchMotor.setAcceleration(100000);
		this.launchMotor.setSpeed(100000);
		this.launchMotor.rotate(180, false);
		Sound.beep();
		
		this.launchMotor.rotate(-85, false);
		
	
	}

}
