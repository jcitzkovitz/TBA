package tba;

import java.util.Map;

import tba.USLocalizerV2.LocalizationType;
import tba.WifiConnection;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class Play {

	/* Initialize all motor and sensor fields */
	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	private static final Port colorPort = LocalEV3.get().getPort("S1");	
	private static final Port usPort = LocalEV3.get().getPort("S2");
	
	/* Instantiate Wifi related fields */
	private static final String SERVER_IP = "192.168.2.5";
	private static final int TEAM_NUMBER = 4;
	private static final boolean ENABLE_DEBUG_WIFI_PRINT = true;
	/* Set up navigation, odometer, odometer correction and 
	 * odometry display objects*/
	private static Odometer odo = new Odometer(leftMotor,rightMotor,30,true);
	private static Navigation nav = new Navigation(odo);
	
	private static final double TILE_LENGTH = 30.48;
	
	@SuppressWarnings("rawtypes")
	public static void main (String[] args)
	{

		/* Retrieve information from wifi */
		WifiConnection conn = new WifiConnection(SERVER_IP, TEAM_NUMBER, ENABLE_DEBUG_WIFI_PRINT);

		int fwdTeam = 0;
		int defTeam = 0;
		double defZoneSizeW1 = 0;
		double defZoneSizeW2 = 0;
		int fwdStartCorner = 0;
		int defStartCorner = 0;
		String despenserOrientation = "";
		double dispX = 0;
		double dispY = 0;
		double shootingDistance = 0;
		
		try {
			/*
			 * getData() will connect to the server and wait until the user/TA
			 * presses the "Start" button in the GUI on their laptop with the
			 * data filled in. Once it's waiting, you can kill it by
			 * pressing the upper left hand corner button (back/escape) on the EV3.
			 * getData() will throw exceptions if it can't connect to the server
			 * (e.g. wrong IP address, server not running on laptop, not connected
			 * to WiFi router, etc.). It will also throw an exception if it connects 
			 * but receives corrupted data or a message from the server saying something 
			 * went wrong. For example, if TEAM_NUMBER is set to 1 above but the server expects
			 * teams 17 and 5, this robot will receive a message saying an invalid team number 
			 * was specified and getData() will throw an exception letting you know.
			 */
			Map data = conn.getData();

			fwdTeam = ((Long) data.get("FWD_TEAM")).intValue();
			defTeam = ((Long) data.get("DEF_TEAM")).intValue();
			defZoneSizeW1 = TILE_LENGTH*((Long) data.get("w1")).intValue();
			defZoneSizeW2 = TILE_LENGTH*((Long) data.get("w2")).intValue();
			fwdStartCorner = ((Long) data.get("FWD_CORNER")).intValue();
			defStartCorner = ((Long) data.get("DEF_CORNER")).intValue();
			despenserOrientation = (String) data.get("omega");
			dispX = TILE_LENGTH*((Long) data.get("bx")).intValue();
			dispY = TILE_LENGTH*((Long) data.get("by")).intValue();
			shootingDistance = TILE_LENGTH*((Long) data.get("d1")).intValue();

		} catch (Exception e) {
			System.err.println("Error: " + e.getMessage());
		}
		
		/* Set up odometry display*/
		final TextLCD t = LocalEV3.get().getTextLCD();
		OdometryDisplay odoDisplay = new OdometryDisplay(odo,t);
		odo.start();
		
		/* With the information retrieved from wifi, localize,
		 * set the start position to the corresponding corner,
		 * notify robot of game position, and play accordingly */
		
		// Instantiate us and light sensor required variables
		
		// Setup us sensor
		SensorModes usSensor = new EV3UltrasonicSensor(usPort);		// usSensor is the instance
		SampleProvider usDistance = usSensor.getMode("Distance");	// usDistance provides samples from this instance
		float[] usData = new float[usDistance.sampleSize()];		// usData is the buffer in which data are returned

		// Setup light sensor
		@SuppressWarnings("resource")
		SensorModes colorSensor = new EV3ColorSensor(colorPort);
		SampleProvider colorValue = colorSensor.getMode("Red");			// colorValue provides samples from this instance
		float[] colorData = new float[colorValue.sampleSize()];			// colorData is the buffer in which data are returned
		
		// Create US and Light Localization objects
		USLocalizerV2 usLoc = new USLocalizerV2(odo,usDistance,usData,nav,LocalizationType.FALLING_EDGE);
		LightLocalizerV3 lightLoc = new LightLocalizerV3(odo,colorValue,colorData,nav);
		
		// Do us Localization
		usLoc.doLocalization();
		
		// Do light localization
		lightLoc.doLocalization();
		
		
		if(fwdTeam == 4)
		{
			// Set the start position for the forward
			setStartPosition(fwdStartCorner);
			
			/* While still playing, continusously go to the 
			 * dispenser, retrieve the ball, go to the 
			 * shooting position and shoot*/
			
			while(true)
			{
			// Travel to the ball dispenser
			nav.travelTo(dispX, dispY);
			
			//Turn to proper orientation
			if(despenserOrientation.equals("N"))
			{
				nav.turnTo(90, true);
			}
			else if(despenserOrientation.equals("E"))
			{
				nav.turnTo(0, true);
			}
			else if(despenserOrientation.equals("S"))
			{
				nav.turnTo(270, true);
			}
			else if(despenserOrientation.equals("W"))
			{
				nav.turnTo(180, true);
			}
			
			// Back up into dispenser to retrieve ball
			
			//TODO
			
			// Travel to the center of the shooting line
			nav.travelTo(6*TILE_LENGTH, shootingDistance);
			
			//Launch
			
			//TODO
			}

		}
		else if(defTeam == 4)
		{
			// Set the start position for the defense
			setStartPosition(defStartCorner);
		}
		
	}
	
	
	public static void setStartPosition(int corner)
	{
		/* Based on the corner given, the starting positions
		 * will be set to a specific position, and from that 
		 * position the robot will navigate to inside the zone
		 * to start*/
		
		if(corner == 1)
		{
			odo.setPosition((new double[] {0,0,0}), (new boolean[] {true,true,true}));
			nav.travelTo(3*TILE_LENGTH, 0);
		}
		else if(corner == 2)
		{
			odo.setPosition((new double[] {10*TILE_LENGTH,0,90}), (new boolean[] {true,true,true}));
			nav.travelTo(7*TILE_LENGTH, 0);
		}
		else if(corner == 3)
		{
			odo.setPosition((new double[] {10*TILE_LENGTH,10*TILE_LENGTH,180}), (new boolean[] {true,true,true}));
			nav.travelTo(7*TILE_LENGTH,10*TILE_LENGTH);
		}
		else if(corner == 4)
		{
			odo.setPosition((new double[] {0,10*TILE_LENGTH,270}), (new boolean[] {true,true,true}));
			nav.travelTo(0, 3*TILE_LENGTH);
		}
	}
	
}
