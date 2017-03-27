package tests;

import tba.LightLocalizerV3;
import tba.Navigation;
import tba.Odometer;
import tba.OdometerCorrectionV2;
import tba.OdometryDisplay;
import tba.USLocalizerV2;
import tba.USLocalizerV2.LocalizationType;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class NavigationTest {

	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	private static final Port colorPortR = LocalEV3.get().getPort("S3");	
	private static final Port colorPortL = LocalEV3.get().getPort("S1");
	private static final Port usPort = LocalEV3.get().getPort("S2");
	
	public static void main(String[] args)
	{
		Odometer odo = new Odometer(leftMotor,rightMotor,30,true);
		Navigation nav = new Navigation(odo);
		
		final TextLCD t = LocalEV3.get().getTextLCD();
		
		OdometryDisplay odoDisplay = new OdometryDisplay(odo,t);
		odoDisplay.start();
		
		// Setup us sensor
				SensorModes usSensor = new EV3UltrasonicSensor(usPort);		// usSensor is the instance
				SampleProvider usDistance = usSensor.getMode("Distance");	// usDistance provides samples from this instance
				float[] usData = new float[usDistance.sampleSize()];		// usData is the buffer in which data are returned

				// Setup light sensor Right and Left
				@SuppressWarnings("resource")
				SensorModes colorSensorR = new EV3ColorSensor(colorPortR);
				SampleProvider colorValueR = colorSensorR.getMode("Red");			// colorValue provides samples from this instance
				float[] colorDataR = new float[colorValueR.sampleSize()];			// colorData is the buffer in which data are returned
				
				SensorModes colorSensorL = new EV3ColorSensor(colorPortL);
				SampleProvider colorValueL = colorSensorL.getMode("Red");			// colorValue provides samples from this instance
				float[] colorDataL = new float[colorValueL.sampleSize()];			// colorData is the buffer in which data are returned
				
				OdometerCorrectionV2 odoCorrection = new OdometerCorrectionV2(odo, colorValueR, colorDataR, colorValueL, colorDataL);
				odoCorrection.start();
				
				// Create US and Light Localization objects
				USLocalizerV2 usLoc = new USLocalizerV2(odo,usDistance,usData,nav,LocalizationType.FALLING_EDGE);
				LightLocalizerV3 lightLoc = new LightLocalizerV3(odo,colorValueR,colorDataR,nav);
				
//				// Do us Localization
//				usLoc.doLocalization();
////				
//////				// Do light localization
//				lightLoc.doLocalization();
		odo.setBaseWidth(11.5);
		nav.travelTo(30.48, 30.48);
		nav.travelTo(60.96, 60.96);
		nav.travelTo(0,30.48);
	}
	
}
