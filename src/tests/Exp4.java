package tests;



import tba.Navigation;
import tba.Odometer;
import lejos.hardware.*;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class Exp4{

	// Static Resources:
	// Left motor connected to output A
	// Right motor connected to output D
	
	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	
	
	public static void main(String[] args) {
		
		int buttonChoice;
      	// setup the odometer and display
		Odometer odometer = new Odometer(leftMotor, rightMotor, 30, true);
		
		//setup Navigator 
		Navigation navigate= new Navigation(odometer);
		
	
		
		//setup the TestRun
		StraightTest test = new StraightTest(odometer, navigate);
		
		//interface to ask what the user wants 
		final TextLCD t = LocalEV3.get().getTextLCD();

		
		do {
			// clear the display
			t.clear();

			// ask the user which test to perform 
			t.drawString(" DRIVE STRAIGHT", 0, 0);
			
			
			buttonChoice = Button.waitForAnyPress();
		     }
		
	 while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT && buttonChoice!= Button.ID_ENTER);
		t.clear();
		LCDInfo lcd = new LCDInfo(odometer, t);
		lcd.start();
		if (buttonChoice == Button.ID_LEFT)
			test.goStraight();
		
		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0);
	}
}
