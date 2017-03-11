package tests;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import tba.Navigation;
import tba.Odometer;

public class Exp5 {
	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	private static final EV3LargeRegulatedMotor liftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
	private static final EV3LargeRegulatedMotor launchMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));
	
	
	public static void main(String[] args) {
		
		int buttonChoice;
      	// setup the odometer and display
		Odometer odometer = new Odometer(leftMotor, rightMotor, 30, true);
		
		//setup Navigator 
		Navigation navigate= new Navigation(odometer);
		
	
		
		//setup the TestRun
		BallLauncher test = new BallLauncher(liftMotor, launchMotor );
		
		//interface to ask what the user wants 
		final TextLCD t = LocalEV3.get().getTextLCD();

		
		do {
			// clear the display
			t.clear();

			// ask the user which test to perform 
			t.drawString(" PRESS ENTER TO START LAUNCH", 0, 0);
			
			
			buttonChoice = Button.waitForAnyPress();
		     }
		
	 while (buttonChoice != Button.ID_ENTER && buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT 
				&& buttonChoice != Button.ID_ESCAPE);;
		t.clear();
		LCDInfo lcd = new LCDInfo(odometer, t);
		lcd.start();
		if (buttonChoice == Button.ID_ENTER)
			test.launch();
		
		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0);
	}
}
