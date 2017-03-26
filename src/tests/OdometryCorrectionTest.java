package tests;

import tba.Navigation;
import tba.Odometer;
import tba.OdometerCorrection;
import tba.OdometryDisplay;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class OdometryCorrectionTest {

	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	private static final Port colorPort = LocalEV3.get().getPort("S1");	

	
public static void main(String[] args) {
		
      	// setup the odometer and display
		Odometer odometer = new Odometer(leftMotor, rightMotor, 30, true);
		
		//setup Navigator 
		Navigation navigate= new Navigation(odometer);
		
//		//setup LightsensorTestor
		@SuppressWarnings("resource")
		SensorModes colorSensor = new EV3ColorSensor(colorPort);
		SampleProvider colorValue = colorSensor.getMode("Red");			// colorValue provides samples from this instance
		float[] colorData = new float[colorValue.sampleSize()];			// colorData is the buffer in which data are returned
//		
		final TextLCD t = LocalEV3.get().getTextLCD();

		
			// clear the display
			t.clear();

			// ask the user which test to perform 
			t.drawString(" L : Straight Run    ", 0, 0);
			t.drawString(" R: Half Square Run  ", 0, 1);
		
//		// setup odometer correction
		OdometerCorrection odoCorrection = new OdometerCorrection(odometer, navigate, colorValue, colorData,1);
		OdometryDisplay odoDisplay = new OdometryDisplay(odometer, t);
		odoCorrection.start();
		odoDisplay.start();
		
		navigate.travelTo(30, 30);
		navigate.travelTo(60, 30);
		navigate.travelTo(0, 0);
		
	}
	
}