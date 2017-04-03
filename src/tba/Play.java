package tba;

import java.util.Map;

import tba.USLocalizerV2.LocalizationType;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

/** 
 * @author Jordan Itzkovitz
 * @author William Wang
 * 
 * The Play class is the main class of which the game will be played.
 * All other classes will be called upon in this class to perform the
 * required tasks.
 * */

public class Play {

	/* Initialize all motor and sensor fields */
	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	private static EV3LargeRegulatedMotor leftCatapultMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));;
	private static EV3LargeRegulatedMotor rightCatapultMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));;
	private static final Port colorPortL = LocalEV3.get().getPort("S1");	
	private static final Port usPortF = LocalEV3.get().getPort("S2");
	private static final Port colorPortR = LocalEV3.get().getPort("S3");	
	private static final Port usPortR = LocalEV3.get().getPort("S4");
	
	/* Instantiate Wifi related fields */
	private static final String SERVER_IP = "192.168.2.10";
	private static final int TEAM_NUMBER = 4;
	private static final boolean ENABLE_DEBUG_WIFI_PRINT = true;
	/* Set up navigation, odometer, odometer correction and 
	 * odometry display objects*/
	private static Odometer odo = new Odometer(leftMotor,rightMotor,30,true);
	private static Navigation nav;
	
	private static final double TILE_LENGTH = 30.48;
	private static final int SLOW = 100;
	private int startCorner;
	public static double dispX = 6*TILE_LENGTH;
	public static double dispY = 0*TILE_LENGTH;
	
	/**
	 * This is the main method for playing the game
	 * */
	@SuppressWarnings("rawtypes")
	public static void main (String[] args)
	{

		/* Retrieve information from wifi */
		WifiConnection conn = new WifiConnection(SERVER_IP, TEAM_NUMBER, ENABLE_DEBUG_WIFI_PRINT);

		int fwdTeam = 4;
		int defTeam = 1;
		double defZoneSizeW1 = 2;
		double defZoneSizeW2 = 2;
		int fwdStartCorner = 1;
		int defStartCorner = 2;
		String despenserOrientation = "S";
		double shootingDistance = 4*TILE_LENGTH;
//		try {
//			/*
//			 * getData() will connect to the server and wait until the user/TA
//			 * presses the "Start" button in the GUI on their laptop with the
//			 * data filled in. Once it's waiting, you can kill it by
//			 * pressing the upper left hand corner button (back/escape) on the EV3.
//			 * getData() will throw exceptions if it can't connect to the server
//			 * (e.g. wrong IP address, server not running on laptop, not connected
//			 * to WiFi router, etc.). It will also throw an exception if it connects 
//			 * but receives corrupted data or a message from the server saying something 
//			 * went wrong. For example, if TEAM_NUMBER is set to 1 above but the server expects
//			 * teams 17 and 5, this robot will receive a message saying an invalid team number 
//			 * was specified and getData() will throw an exception letting you know.
//			 */
//			Map data = conn.getData();
//
//			fwdTeam = ((Long) data.get("FWD_TEAM")).intValue();
//			defTeam = ((Long) data.get("DEF_TEAM")).intValue();
//			defZoneSizeW1 = TILE_LENGTH*((Long) data.get("w1")).intValue();
//			defZoneSizeW2 = TILE_LENGTH*((Long) data.get("w2")).intValue();
//			fwdStartCorner = ((Long) data.get("FWD_CORNER")).intValue();
//			defStartCorner = ((Long) data.get("DEF_CORNER")).intValue();
//			despenserOrientation = (String) data.get("omega");
//			dispX = TILE_LENGTH*((Long) data.get("bx")).intValue();
//			dispY = TILE_LENGTH*((Long) data.get("by")).intValue();
//			shootingDistance = TILE_LENGTH*((Long) data.get("d1")).intValue();
//
//		} catch (Exception e) {
//			System.err.println("Error: " + e.getMessage());
//		}
		
		/* Set up odometry display*/
		final TextLCD t = LocalEV3.get().getTextLCD();
		OdometryDisplay odoDisplay = new OdometryDisplay(odo,t);
		odo.start();
		odoDisplay.start();
		/* With the information retrieved from wifi, localize,
		 * set the start position to the corresponding corner,
		 * notify robot of game position, and play accordingly */
	
		// Instantiate us and light sensor required variables
		
		// Setup us sensor
		SensorModes usSensorF = new EV3UltrasonicSensor(usPortF);		// usSensor is the instance
		SampleProvider usDistanceF = usSensorF.getMode("Distance");	// usDistance provides samples from this instance
		float[] usDataF = new float[usDistanceF.sampleSize()];		// usData is the buffer in which data are returned

		SensorModes usSensorR = new EV3UltrasonicSensor(usPortR);		// usSensor is the instance
		SampleProvider usDistanceR = usSensorR.getMode("Distance");	// usDistance provides samples from this instance
		float[] usDataR = new float[usDistanceR.sampleSize()];		// usData is the buffer in which data are returned
		
//		// Setup light sensor
		@SuppressWarnings("resource")
		SensorModes colorSensorL = new EV3ColorSensor(colorPortL);
		SampleProvider colorValueL = colorSensorL.getMode("Red");			// colorValue provides samples from this instance
		float[] colorDataL = new float[colorValueL.sampleSize()];			// colorData is the buffer in which data are returned
//		
		SensorModes colorSensorR = new EV3ColorSensor(colorPortR);
		SampleProvider colorValueR = colorSensorR.getMode("Red");			// colorValue provides samples from this instance
		float[] colorDataR = new float[colorValueR.sampleSize()];			// colorData is the buffer in which data are returned
		
		nav = new Navigation(odo,usDistanceR, usDataR, usDistanceF, usDataF,6, colorValueR,colorDataR,colorValueL,colorDataL,shootingDistance);
		
		// Create US and Light Localization objects
		USLocalizerV2 usLoc = new USLocalizerV2(odo,usDistanceF,usDataF,nav,LocalizationType.FALLING_EDGE);
		LightLocalizerV4 lightLoc = new LightLocalizerV4(odo,colorValueR,colorDataR,colorValueL,colorDataL,nav);
		
		//Create launcher
		BallLauncher launcher = new BallLauncher(leftCatapultMotor, rightCatapultMotor, (float)(10-shootingDistance/TILE_LENGTH));
		CorrectHeading correctHeading = new CorrectHeading(odo,nav,colorValueR,colorDataR,colorValueL,colorDataL);
		OdometerCorrection odoCorrection = new OdometerCorrection(odo,nav,colorValueR,colorDataR,colorValueL,colorDataL);
		
		odo.setBaseWidth(10.4);
		// Do us Localization
		usLoc.doLocalization();
		
		// Do light localization
		lightLoc.doLocalization();
		
		if(fwdTeam == 4)
		{
			nav.forwardTeam();
			
			// Set correct angle and position to correct position
			setStartPosition(fwdStartCorner);
			odoCorrection.start();
			correctHeading.start();
			
			/* While still playing, continusously go to the 
			 * dispenser, retrieve the ball, go to the 
			 * shooting position and shoot*/
			
			while(true)
			{
			// Travel to the ball dispenser
			if(despenserOrientation.equals("N")){
				nav.travelTo(dispX-TILE_LENGTH/2, dispY-TILE_LENGTH/2);
			}
			else if(despenserOrientation.equals("E"))
			{
				nav.travelTo(dispX-TILE_LENGTH/2, dispY-TILE_LENGTH/2);
			}
			else if(despenserOrientation.equals("S"))
			{
				nav.travelTo(dispX-TILE_LENGTH/2, dispY+TILE_LENGTH/2);
			}
			else if(despenserOrientation.equals("W"))
			{
				nav.travelTo(dispX+TILE_LENGTH/2, dispY-TILE_LENGTH/2);
			}
				
			//Lower launcher motors
//			leftCatapultMotor.setSpeed(25);
//			rightCatapultMotor.setSpeed(25);
//			
//			leftCatapultMotor.rotate(70,true);
//			rightCatapultMotor.rotate(70,false);
//			
			//Localize at ball dispenser line
			nav.dispenserLocalization();
			
			//Turn to proper orientation
			boolean xAxis = true;
			if(despenserOrientation.equals("N"))
			{
				xAxis=false;
				nav.turnTo(270, true);
			}
			else if(despenserOrientation.equals("E"))
			{
				xAxis=true;
				nav.turnTo(180, true);
			}
			else if(despenserOrientation.equals("S"))
			{
				xAxis=false;
				nav.turnTo(90, true);
			}
			else if(despenserOrientation.equals("W"))
			{
				xAxis=true;
				nav.turnTo(0, true);
			}
			
			//Re set position on odmeter to position of ball dispenser coordinate
			odo.setPosition((new double[] {dispX,dispY,0}), (new boolean[] {true,true,false}));
			
			//Back up into dispenser to retrieve ball
			nav.drive(5,xAxis,SLOW,false);
			//Wait for ball to be dispensed
			try{Thread.sleep(7000);}catch(Exception e){}
			
			// Travel to the center of the shooting line
			nav.travelTo(3*TILE_LENGTH, 6*TILE_LENGTH-shootingDistance);
			
			
			//Launch
			nav.turnTo(90,true);
			launcher.launch();
			
			}

		}
		else if(defTeam == 4)
		{
			// Set the start position for the defense
			setStartPosition(defStartCorner);
			
			correctHeading.start();
			odoCorrection.start();
			
			nav.defenseTeam();
			nav.travelTo(5*TILE_LENGTH, 8*TILE_LENGTH);
			
		}
		
	}
	
	
	public static void setStartPosition(int corner)
	{
		/* Based on the corner given, the starting positions
		 * will be set to a specific position, and from that 
		 * position the robot will navigate to inside the zone
		 * to start*/
		double turnAngle = 0;
		if(corner == 1)
		{
			odo.setPosition((new double[] {0,0,0}), (new boolean[] {true,true,true}));
			turnAngle = 90;
		}
		else if(corner == 2)
		{
			odo.setPosition((new double[] {10*TILE_LENGTH,0,90}), (new boolean[] {true,true,true}));
			turnAngle = 90;
		}
		else if(corner == 3)
		{
			odo.setPosition((new double[] {10*TILE_LENGTH,10*TILE_LENGTH,180}), (new boolean[] {true,true,true}));
			turnAngle = 270;
		}
		else if(corner == 4)
		{
			odo.setPosition((new double[] {0,10*TILE_LENGTH,270}), (new boolean[] {true,true,true}));
			turnAngle = 270;
		}
		
		nav.drive(TILE_LENGTH/2,true,SLOW,true);
		nav.setSpeeds(0, 0);
		nav.turnTo(turnAngle, true);
		try{Thread.sleep(500);}catch(Exception e){}
	}
	
}
