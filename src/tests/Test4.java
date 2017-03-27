package tests;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import tba.BallLauncher;
import tba.Odometer;

public class Test4 {
	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	private static final EV3LargeRegulatedMotor leftCatapultMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
	private static final EV3LargeRegulatedMotor rightCatapultMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));
	
	
	public static void main(String[] args) {
		// setup the odometer and display
		Odometer odometer = new Odometer(leftMotor, rightMotor, 30, true);
		//setup the ballLauncher
		BallLauncher test = new BallLauncher(leftCatapultMotor, rightCatapultMotor,1);
		test.launch();
		
		int buttonChoice = Button.ID_ENTER;
		while(buttonChoice != Button.ID_ESCAPE){
	      
			
			//interface to ask what the user wants 
			final TextLCD t = LocalEV3.get().getTextLCD();
	
			
			do {
				// clear the display
				t.clear();
	
				// ask the user which test to perform 
				t.drawString(" PRESS ENTER TO START LAUNCH", 0, 0);
				
				
				buttonChoice = Button.waitForAnyPress();
				
			} while (buttonChoice != Button.ID_ENTER && buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT 
					&& buttonChoice != Button.ID_ESCAPE);
			t.clear();
			LCDInfo lcd = new LCDInfo(odometer, t);
			lcd.start();
			if (buttonChoice == Button.ID_ENTER)
				test.launch();
		}
		System.exit(0);
	}
}
