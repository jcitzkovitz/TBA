package tests;

import tba.Navigation;
import tba.Odometer;
import tba.OdometryDisplay;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class NavigationTest {

	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	
	public static void main(String[] args)
	{
		Odometer odo = new Odometer(leftMotor,rightMotor,30,true);
		Navigation nav = new Navigation(odo);
		
		final TextLCD t = LocalEV3.get().getTextLCD();
		
		OdometryDisplay odoDisplay = new OdometryDisplay(odo,t);
		
		odoDisplay.start();

		nav.travelTo(30, 30);
		nav.travelTo(60, 60);
		nav.travelTo(0,30);
	}
	
}
