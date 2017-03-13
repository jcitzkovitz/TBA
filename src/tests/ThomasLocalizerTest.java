package tests;



import lejos.hardware.*;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.*;
import lejos.robotics.SampleProvider;
import tba.LightLocalizer;
import tba.Navigation;
import tba.Odometer;
import tba.USLocalizer;
import tba.USLocalizer.LocalizationType;

public class ThomasLocalizerTest {

	// Static Resources:
	// Left motor connected to output A
	// Right motor connected to output D
	// Ultrasonic sensor port connected to input S1
	// Color sensor port connected to input S2
	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	private static final Port usPort = LocalEV3.get().getPort("S1");		
	private static final Port lightPort = LocalEV3.get().getPort("S2");		
    
	
	public static void main(String[] args) {
		int buttonChoice;
		// setup the odometer and display
				Odometer odometer = new Odometer(leftMotor, rightMotor, 30, true);
				
				//setup Navigator 
				Navigation navigate= new Navigation(odometer);
				
				//setup us sensor
				SensorModes usSensor = new EV3UltrasonicSensor(usPort);		// usSensor is the instance
				SampleProvider usDistance = usSensor.getMode("Distance");	// usDistance provides samples from this instance
				float[] usData = new float[usDistance.sampleSize()];		// usData is the buffer in which data are returned

				
				//setup light sensor
				@SuppressWarnings("resource")
				SensorModes colorSensor = new EV3ColorSensor(lightPort);
				SampleProvider colorValue = colorSensor.getMode("Red");			// colorValue provides samples from this instance
				float[] colorData = new float[colorValue.sampleSize()];			// colorData is the buffer in which data are returned
				
				USLocalizer usLoc = new USLocalizer(odometer, navigate, usDistance, usData,LocalizationType.FALLING_EDGE);
		
				LightLocalizer lightLoc = new LightLocalizer(odometer, navigate,  colorData, colorValue, leftMotor, rightMotor);
	
	
		//interface to ask what the user wants 
		final TextLCD t = LocalEV3.get().getTextLCD();
		

		
		//Ask the user what he wants to use
		do {
			// clear the display
			t.clear();

			// ask the user whether the motors should drive in a square or float
			t.drawString("< Left          ", 0, 0);
			t.drawString("                     ", 0, 1);
			t.drawString(" localization  ", 0, 2);
			t.drawString("   test            ", 0, 3);
			t.drawString("                   ", 0, 4);

			buttonChoice = Button.waitForAnyPress();
		}
		
		while (buttonChoice != Button.ID_LEFT
					&& buttonChoice != Button.ID_RIGHT);

		   buttonChoice = Button.waitForAnyPress();
	
			if (buttonChoice == Button.ID_LEFT) { 
			usLoc.doLocalization();
				lightLoc.doLocalization();
				
				buttonChoice = Button.waitForAnyPress();
			}
		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0);	
		
	}

}
