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

public class Exp5 {

	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));

	
public static void main(String[] args) {
		
		
      	// setup the odometer and display
		Odometer odometer = new Odometer(leftMotor, rightMotor, 30, true);
		
		//setup Navigator 
		Navigation navigate= new Navigation(odometer);
		
		navigate.travelTo(30, 30);
		navigate.travelTo(60, 0);
}
	
	
}
