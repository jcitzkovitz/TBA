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
		//lift the ball

		this.leftCatapultMotor.setSpeed(20);
		this.rightCatapultMotor.setSpeed(20);
		this.leftCatapultMotor.rotate(-30,true);
		this.rightCatapultMotor.rotate(-30, false);
		Sound.beep();
		
		//launch the ball

		this.leftCatapultMotor.setAcceleration(2200);
		this.rightCatapultMotor.setAcceleration(2200);
		this.leftCatapultMotor.setSpeed(3000);
		this.rightCatapultMotor.setSpeed(3000);
		this.leftCatapultMotor.rotate(-10, true);
		this.rightCatapultMotor.rotate(-10,false);
		Sound.twoBeeps();
		//return to the initial position
		this.leftCatapultMotor.setAcceleration(500);
		this.rightCatapultMotor.setAcceleration(500);
		this.leftCatapultMotor.setSpeed(90);
		this.rightCatapultMotor.setSpeed(90);
		this.leftCatapultMotor.rotate(40, true);
		this.rightCatapultMotor.rotate(40, false);
		Sound.twoBeeps();
		
		}
	}

