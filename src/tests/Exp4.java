package tests;



import tba.Navigation;
import tba.Odometer;
import tba.OdometryDisplay;
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
		
		final TextLCD t = LocalEV3.get().getTextLCD();
		OdometryDisplay disp = new OdometryDisplay(odometer,t);
		disp.start();
		
//		//setup the TestRun
//		StraightTest test = new StraightTest(odometer, navigate);
//		test.goStraight();
		
		RotationTest test2 = new RotationTest(odometer,navigate);
		test2.rotate(271);
		//interface to ask what the user wants 

		
		
		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0);
	}
}
