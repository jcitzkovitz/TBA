package tests;

import tba.LightLocalization;
import tba.LightLocalizer;
import tba.Navigation;
import tba.Odometer;
import tba.USLocalizer;
import tba.USLocalizer.LocalizationType;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;



public class LocalizationTest {

	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	private static final Port lightPort = LocalEV3.get().getPort("S1");	
	private static final Port usPort = LocalEV3.get().getPort("S2");
	private static double leftRadius =2.1;
	private static double rightRadius=2.05;
	private static double width=10.4;
	
	
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
				usLoc.doLocalization();
				
				LightLocalization lightLoc = new LightLocalization(odometer, colorValue, colorData, navigate, 1);
				lightLoc.doLocalization();
				
				
				
	}
	
}
