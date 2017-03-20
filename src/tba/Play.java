package tba;

import java.util.Map;

import tba.WifiConnection;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;

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
	
	@SuppressWarnings("rawtypes")
	public static void main (String[] args)
	{
		/* Retrieve information from wifi */
		
		WifiConnection conn = new WifiConnection(SERVER_IP, TEAM_NUMBER, ENABLE_DEBUG_WIFI_PRINT);

		int fwdTeam = 0;
		int defTeam = 0;
		int defZoneSizeW1 = 0;
		int defZoneSizeW2 = 0;
		int fwdStartCorner = 0;
		int defStartCorner = 0;
		String despenserOrientation = "";
		int dispX = 0;
		int dispY = 0;
		
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
			defZoneSizeW1 = ((Long) data.get("w1")).intValue();
			defZoneSizeW2 = ((Long) data.get("w2")).intValue();
			fwdStartCorner = ((Long) data.get("FWD_CORNER")).intValue();
			defStartCorner = ((Long) data.get("DEF_CORNER")).intValue();
			despenserOrientation = (String) data.get("omega");
			dispX = ((Long) data.get("bx")).intValue();
			dispY = ((Long) data.get("by")).intValue();
						

		} catch (Exception e) {
			System.err.println("Error: " + e.getMessage());
		}
		
		
		if(fwdTeam == 4)
		{
			
		}
		else if(defTeam == 4)
		{
			
		}
		
	}
	
}
