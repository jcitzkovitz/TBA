package tba;

import lejos.hardware.Sound;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

/**
 * The USLocalizerV2 class is the second version of USLocalization. It uses 
 * ultra-sonic sensors to locate the walls surrounding the robot, where this
 * information is used to locate the 0 degree angle.*/

public class USLocalizerV3 {
	public enum LocalizationType { FALLING_EDGE, RISING_EDGE };
	public static float ROTATION_SPEED = 60;
	
	private Odometer odo;
	private SampleProvider usSensorL;
	private float[] usDataL;

	private LocalizationType locType;
	public Navigation navigation; 
	
	private static final int BANDWIDTH	= 30;
	
	
	public USLocalizerV3(Odometer odo,  SampleProvider usSensorL, float[] usDataL,SampleProvider usSensorM, float[] usDataM,SampleProvider usSensorR, float[] usDataR, Navigation navigation, LocalizationType fallingEdge) {
		this.odo = odo;
		this.usSensorL = usSensorL;
		this.usDataL = usDataL;
		this.locType = fallingEdge;
		this.navigation = navigation;
	}
	
	public void doLocalization() {
		
		// Angle variables
		double angleAL, angleBL, angleAM, angleBM, angleAR, angleBR;
		
		// Angle used to for ending orientation
		double orientationAngleL = 0;
		double orientationAngleR = 0;
		double orientationAngleM = 0;
		
		if (locType == LocalizationType.FALLING_EDGE) {
			
			// Rotate the robot clockwise until there is no wall
			// This is done in the case that the robot begins looking
			// at a wall. We want the robot to begin its readings while
			// it's not facing a wall.
			while(getFilteredData() < BANDWIDTH)
			{
				rotateCW();
			}
			
			// Delay the process in order to avoid bad readings
			Delay.msDelay(1000);
			
			// Rotate the robot clockwise until a wall is detected
			while (getFilteredData() >= BANDWIDTH)
			{	
				rotateCW();
			}
			
			// Stop the robot's motion
			navigation.setSpeeds(0, 0);
			
			// Hold this last angle which will be used to calculate the final
			// orientation angle
			angleAL = odo.getAng();
			
			// Delay the process in order to avoid bad readings
			Delay.msDelay(1000);
			
			// Rotate counter clockwise until no wall is seen (for the same reason
			// as mentioned above)
			while(getFilteredData() < BANDWIDTH)
			{
				rotateCCW();
			}
			
			// Delay the process in order to avoid bad readings
			Delay.msDelay(1000);
			
			// Rotate counter clockwise until a wall is seen
			while (getFilteredData() >= BANDWIDTH)
			{
				rotateCCW();
			}
			
			// Stop the robot's motion
			navigation.setSpeeds(0, 0);
			
			// Hold this last angle which will be used to calculate the final
			// orientation angle
			angleBL = odo.getAng();
			
			// Delay the process in order to avoid bad readings
			Delay.msDelay(1000);
			
			// Calculate the orientation angle for final positioning
			orientationAngleL = calculateFinalOrientationAngle(angleAL,angleBL)+odo.getAng();
			
			// Update the odometer position
			double[] finalOrientation = {0.0, 0.0, orientationAngleL};
			boolean[] setAllPositions = {true,true,true};
			odo.setPosition(finalOrientation, setAllPositions);
			
			// Rotate to angle 0
			navigation.turnTo(0, true);
			
		} else {
			/*
			 * The robot should turn until it sees the wall, then look for the
			 * "rising edges:" the points where it no longer sees the wall.
			 * This is very similar to the FALLING_EDGE routine, but the robot
			 * will face toward the wall for most of it.
			 */
			
			// Rotate the robot clockwise until it sees a wall
			while(getFilteredData() > BANDWIDTH)
			{
				rotateCW();
			}
			
			// Delay the process in order to avoid bad readings
			Delay.msDelay(1000);
						
			// Rotate the robot clockwise until a wall is no longer seen
			while (getFilteredData() <= BANDWIDTH)
			{
				rotateCW();
			}
			
			// Stop the robot's motion
			navigation.setSpeeds(0, 0);
			
			// Hold this last angle which will be used to calculate the final
			// orientation angle
			angleAL = odo.getAng();
			
			// Delay the process in order to avoid bad readings
			Delay.msDelay(1000);
			
			// Rotate counterclockwise until a wall is detected			
			while(getFilteredData() > BANDWIDTH)
			{
				rotateCCW();
			}
			
			// Delay the process in order to avoid bad readings
			Delay.msDelay(1000);
			
			// Rotate counterclockwise until no wall is seen 
			while (getFilteredData() <= BANDWIDTH)
			{
				rotateCCW();
			}
			
			// Stop the robot's motion
			navigation.setSpeeds(0, 0);
			
			// Hold this last angle which will be used to calculate the final
			// orientation angle
			angleBL = odo.getAng();
			
			// Delay the process in order to avoid bad readings
			Delay.msDelay(1000);
			
			// Calculate the orientation angle for final positioning
			orientationAngleL = calculateFinalOrientationAngle(angleBL,angleAL)+odo.getAng();
			
			// Update the odometer position
			double[] finalOrientation = {0.0, 0.0, orientationAngleL};
			boolean[] setAllPositions = {true,true,true};
			odo.setPosition(finalOrientation, setAllPositions);
			
			// Rotate to angle 0
			navigation.turnTo(0, true);

		}
	}
	
	private float getFilteredData() 
	{
		
		usSensorL.fetchSample(usDataL, 0);
		float distance = usDataL[0]*100;
		
		int filterValue = 50;
		
		// If the usSensor reads anything greater then filterValue, set the distance
		// to the filterValue
		if (distance >= filterValue)
		{
			distance = filterValue;
		}
		return distance;
	}
	
	// Counter clockwise rotation
	private  void rotateCCW()
	{
		navigation.setSpeeds(-ROTATION_SPEED, ROTATION_SPEED);
	}

	// Clockwise rotation
	private  void rotateCW()
	{
		navigation.setSpeeds(ROTATION_SPEED, -ROTATION_SPEED);
	}
	
	private static double calculateFinalOrientationAngle(double angleA, double angleB){ // calculation of heading as shown in the tutorial
		
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
