package tba;

import lejos.hardware.Sound;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

/**
 * @author William Wang
 * 
 * The USLocalizerV2 class is the second version of USLocalization. It uses 
 * ultra-sonic sensors to locate the walls surrounding the robot, where this
 * information is used to locate the 0 degree angle.*/

public class USLocalizerV2 {
	public enum LocalizationType { FALLING_EDGE, RISING_EDGE };
	public static float ROTATION_SPEED = 150;
	
	private Odometer odo;
	private SampleProvider usSensorF;
	private float[] usDataF;
	private LocalizationType locType;
	public Navigation nav; 
	
	private static final int BANDWIDTH	= 30;
	
	/**
	 * USLocalizerV2 constructor 
	 * 
	 * @param 	odo 	Odometer object
	 * @param 	usSensorF 	Sample provider for the front usSensor
	 * @param 	usDataF 	Array of float values used to hold us sensor data
	 * @param 	fallingEdge 	Type of localization that will be used
	 * @param 	nav 	Navigation object
	 * */
	public USLocalizerV2(Odometer odo,  SampleProvider usSensorF, float[] usDataF, Navigation nav, LocalizationType fallingEdge) {
		this.odo = odo;
		this.usSensorF = usSensorF;
		this.usDataF = usDataF;
		this.locType = fallingEdge;
		this.nav = nav;
	}
	
	/**
	 * doLocalization() performs localization using us sensors. This method only works in corners and
	 * turns to '0' degrees by the end
	 * 
	 * @return		void
	 * */
	public void doLocalization() {
		
		// Angle variables
		double angleA, angleB;
		
		// Angle used to for ending orientation
		double orientationAngle = 0;
		
		if (locType == LocalizationType.FALLING_EDGE) {
			
			// Rotate the robot clockwise until there is no wall
			// This is done in the case that the robot begins looking
			// at a wall. We want the robot to begin its readings while
			// it's not facing a wall.
			while(getFilteredDataF() < BANDWIDTH)
			{
				rotateCW();
			}
			
			// Delay the process in order to avoid bad readings
			Delay.msDelay(1000);
			
			// Rotate the robot clockwise until a wall is detected
			while (getFilteredDataF() >= BANDWIDTH)
			{	
				rotateCW();
			}
			
			// Stop the robot's motion
			nav.setSpeeds(0, 0);
			
			// Hold this last angle which will be used to calculate the final
			// orientation angle
			angleA = odo.getAng();
			
			// Delay the process in order to avoid bad readings
			Delay.msDelay(1000);
			
			// Rotate counter clockwise until no wall is seen (for the same reason
			// as mentioned above)
			int i = 0;
			while(getFilteredDataF() < BANDWIDTH)
			{
				rotateCCW();
				if(i==0)
					try{Thread.sleep(1000);}catch(Exception e){}
				i++;
			}
			
			// Delay the process in order to avoid bad readings
			Delay.msDelay(1000);
			
			i=0;
			// Rotate counter clockwise until a wall is seen
			while (getFilteredDataF() >= BANDWIDTH)
			{
				rotateCCW();
				if(i==0)
					try{Thread.sleep(1000);}catch(Exception e){}
				i++;
			}
			
			// Stop the robot's motion
			nav.setSpeeds(0, 0);
			
			// Hold this last angle which will be used to calculate the final
			// orientation angle
			angleB = odo.getAng();
			
			// Delay the process in order to avoid bad readings
			Delay.msDelay(1000);
			
			// Calculate the orientation angle for final positioning
			orientationAngle = calculateFinalOrientationAngle(angleA,angleB)+odo.getAng();
			
			// Update the odometer position
			double[] finalOrientation = {0.0, 0.0, orientationAngle};
			boolean[] setAllPositions = {true,true,true};
			odo.setPosition(finalOrientation, setAllPositions);
			
			// Rotate to angle 0
			nav.turnTo(0, true);
			
			// Stop the robot's motion
			nav.setSpeeds(0, 0);
			
			try{Thread.sleep(1000);}catch(Exception e){}
			
		} else {
			/*
			 * The robot should turn until it sees the wall, then look for the
			 * "rising edges:" the points where it no longer sees the wall.
			 * This is very similar to the FALLING_EDGE routine, but the robot
			 * will face toward the wall for most of it.
			 */
			
			//Rotate the robot clockwise until it sees a wall
			while(getFilteredDataF() > BANDWIDTH)
			{
				rotateCW();
			}
			
			//Delay the process in order to avoid bad readings
			Delay.msDelay(1000);
						
			//Rotate the robot clockwise until a wall is no longer seen
			while (getFilteredDataF() <= BANDWIDTH)
			{
				rotateCW();
			}
			
			//Stop the robot's motion
			nav.setSpeeds(0, 0);
			
			//Hold this last angle which will be used to calculate the final
			//orientation angle
			angleA = odo.getAng();
			
			//Delay the process in order to avoid bad readings
			Delay.msDelay(1000);
			
			//Rotate counterclockwise until a wall is detected			
			while(getFilteredDataF() > BANDWIDTH)
			{
				rotateCCW();
			}
			
			//Delay the process in order to avoid bad readings
			Delay.msDelay(1000);
			
			//Rotate counterclockwise until no wall is seen 
			while (getFilteredDataF() <= BANDWIDTH)
			{
				rotateCCW();
			}
			
			//Stop the robot's motion
			nav.setSpeeds(0, 0);
			
			//Hold this last angle which will be used to calculate the final
			//orientation angle
			angleB = odo.getAng();
			
			//Delay the process in order to avoid bad readings
			Delay.msDelay(1000);
			
			//Calculate the orientation angle for final positioning
			orientationAngle = calculateFinalOrientationAngle(angleB,angleA)+odo.getAng();
			
			//Update the odometer position
			double[] finalOrientation = {0.0, 0.0, orientationAngle};
			boolean[] setAllPositions = {true,true,true};
			odo.setPosition(finalOrientation, setAllPositions);
			
			//Rotate to angle 0
			nav.turnTo(0, true);

		}
	}
	
	/**
	 * Get the the us Sensor distance value from the front us sensor
	 * 
	 * @return		Front us sensor value
	 * */
	private float getFilteredDataF() 
	{
		
		usSensorF.fetchSample(usDataF, 0);
		float distance = usDataF[0]*100;
		
		int filterValue = 50;
		
		//If the usSensor reads anything greater then filterValue, set the distance
		//to the filterValue
		if (distance >= filterValue)
		{
			distance = filterValue;
		}
		return distance;
	}
	
	/**
	 * Rotate counterclockwise
	 * 
	 * @return		void
	 * */
	private  void rotateCCW()
	{
		nav.setSpeeds(-ROTATION_SPEED, ROTATION_SPEED);
	}

	/**
	 * Rotate clockwise
	 * 
	 * @return		void
	 * */
	private  void rotateCW()
	{
		nav.setSpeeds(ROTATION_SPEED, -ROTATION_SPEED);
	}
	
	/**
	 * Calculation of heading as shown in the tutorial
	 * 
	 * @return		Final angle of which the robot needs to rotate to face '0' degrees
	 * */
	private static double calculateFinalOrientationAngle(double angleA, double angleB){ 
		
		double orientationAngle = 0;
		
		if (angleA < angleB)
		{
			orientationAngle = 45-(angleA+angleB)/2 ;
		}else 
		{
			orientationAngle = 225-(angleA+angleB)/2 ;	
		}
		
		return orientationAngle;
	}
}
