package tests;



import lejos.hardware.*;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class Exp1{

	// Static Resources:
	// Left motor connected to output A
	// Right motor connected to output D
	
	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	private static final Port colorPort = LocalEV3.get().getPort("S1");	
	private static double leftRadius =2.1;
	private static double rightRadius=2.05;
	private static double width=10.4;
	
	public static void main(String[] args) {
		
		int buttonChoice;
      	// setup the odometer and display
		Odometer odometer = new Odometer(leftMotor, rightMotor, 30, true);
		
		//setup Navigator 
		Navigation navigate= new Navigation(odometer);
		
		//setup LightsensorTestor
		@SuppressWarnings("resource")
		SensorModes colorSensor = new EV3ColorSensor(colorPort);
		SampleProvider colorValue = colorSensor.getMode("ColorID");			// colorValue provides samples from this instance
		float[] colorData = new float[colorValue.sampleSize()];			// colorData is the buffer in which data are returned
		LightsensorTester lightTest = new LightsensorTester(odometer, colorValue, colorData, navigate);
		
		//setup the TestRun
		TestRun test = new TestRun(navigate);
		
		//interface to ask what the user wants 
		final TextLCD t = LocalEV3.get().getTextLCD();

		
		do {
			// clear the display
			t.clear();

			// ask the user which test to perform 
			t.drawString(" L : Straight Run    ", 0, 0);
			t.drawString(" R: Half Square Run  ", 0, 1);
			
			buttonChoice = Button.waitForAnyPress();
		     }
		
	 while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT && buttonChoice!= Button.ID_ENTER);
		t.clear();
		LCDInfo lcd = new LCDInfo(odometer, t);
		lcd.start();
		if (buttonChoice == Button.ID_LEFT)
			test.StraightTest();
		else if(buttonChoice == Button.ID_ENTER){
			lightTest.test();
			
		}
		else
			(new Thread() {
				public void run() {
					SquareDriver.drive(leftMotor, rightMotor, leftRadius, rightRadius, width);
				}
			}).start();
		
		
		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0);
	}
}
