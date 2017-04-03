package tba;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

/**
 * @author Jordan Itzkovitz and William Wang
 * 
 * The LightLocalizrV4 class performs light localization using two color sensors located on the right
 * and left sides at the front of the robot*/
public class LightLocalizerV4 {

	private Odometer odo;
	private SampleProvider colorSensorR;
	private float[] colorDataR;	
	private SampleProvider colorSensorL;
	private float[] colorDataL;	
	private Navigation nav;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	private float SPEED = 100;
	public static double lightSensorDistance = 2.1;
	
	/**
	 * LightLocalizerV4 constructor class
	 * @param odo Odometer object in charge of providing Odometer class methods
	 * @param nav Navigation object in charge of providing Navigation class methods
	 * @param colorSensorR Sample provider variable for the right color sensor
	 * @param colorSensorL Sample provider variable for the left color sensor
	 * @param colorDataR Float array to hold right color sensor data
	 * @param colorDataL Float array to hold left color sensor data
	 * */
	
	public LightLocalizerV4(Odometer odo, SampleProvider colorSensorR, float[] colorDataR, SampleProvider colorSensorL, float[] colorDataL, Navigation nav) {
		this.odo = odo;
		this.colorSensorR = colorSensorR;
		this.colorDataR = colorDataR;
		this.colorSensorL = colorSensorL;
		this.colorDataL = colorDataL;
		this.nav= nav;
		EV3LargeRegulatedMotor[] motors = this.odo.getMotors();
		this.leftMotor = motors[0];
		this.rightMotor = motors[1];
	}
	
	/**
	 * doLocalization() performs localization using the knowledge of usSensorV2 localization which
	 * tells the robot where the '0' degree angle is facing. From there, it turns 90 degrees to face the y
	 * direction, travels forward until it sees a black line with both the left and right color sensors and
	 * performs CorrectHeading. It then tells the robot to turn to 0 degrees and perform the same motion
	 * as done in the y direction. 
	 * 
	 * @return void
	 * */
	
	public void doLocalization() {
		
		//Minimum light value used to track black lines
		double minLight =0.3;
		
		//Booleans which tells the program which sensor hit the black line first
		boolean rightHit = false,leftHit = false;
		
		//Hold the first hit value
		double firstHit = 0;
		
		//Overall correction needed
		double correction = 0;

		// Turn to positive y position
		nav.turnTo(90, true);
		
		// travel in the y direction until both light sensors sense a black line, and perform correction
		while(true)
		{
			nav.setSpeeds(SPEED,SPEED);
			if(getColorDataR()<minLight)
			{
				if(leftHit)
				{
					correction = this.odo.getY()-firstHit+.5;
					correction = Math.toDegrees(Math.asin(correction/odo.getBaseWidth()));
					correctHeading(false, correction);
					leftHit=false;
					rightHit=false;
					break;
				}
				else{
					rightHit = true;
					firstHit = this.odo.getY();
				}
			}

			if(getColorDataL()<minLight)
			{

				if(rightHit)
				{
					correction = this.odo.getY()-firstHit+.5;
					correction = Math.toDegrees(Math.asin(correction/odo.getBaseWidth()));
					correctHeading(true, correction);
					leftHit=false;
					rightHit=false;
					break;
				}
				else{
					leftHit = true;
					firstHit = this.odo.getY();
				}
			}
		}
		
		//Drive the distance between the light sensor and the wheels to get the wheels on the black line
		double currentY = odo.getY();
		while(Math.abs(currentY-odo.getY()) < lightSensorDistance)
		{
			nav.setSpeeds(SPEED,SPEED);
		}
		
		nav.setSpeeds(0,0);
		try{Thread.sleep(500);}catch(Exception e){}
		
		 //Turn to positive x direction and travel until both sensors sense a black line and perform correction
		 nav.turnTo(0, true);
			while(true)
			{
				nav.setSpeeds(SPEED,SPEED);
				if(getColorDataR()<minLight)
				{
					if(leftHit)
					{
						correction = this.odo.getX()-firstHit+.5;
						correction = Math.toDegrees(Math.asin(correction/odo.getBaseWidth()));
						correctHeading(false, correction);
						leftHit=false;
						rightHit=false;
						break;
					}
					else{
						rightHit = true;
						firstHit = this.odo.getX();
					}
				}

				if(getColorDataL()<minLight)
				{

					if(rightHit)
					{
						correction = this.odo.getX()-firstHit+.5;
						correction = Math.toDegrees(Math.asin(correction/odo.getBaseWidth()));
						correctHeading(true, correction);
						leftHit=false;
						rightHit=false;
						break;
					}
					else{
						leftHit = true;
						firstHit = this.odo.getX();
					}
				}
			}
			
			//Drive the distance between the light sensor and the wheels to get the wheels on the black line
			double currentX = odo.getX();
			while(Math.abs(currentX-odo.getX()) < lightSensorDistance)
			{
				nav.setSpeeds(SPEED,SPEED);
			}
			nav.setSpeeds(0,0);
			try{Thread.sleep(500);}catch(Exception e){}
			
	}
	
	/**
	 * Get the the light strength from the right color sensor
	 * 
	 * */
	public float getColorDataR(){
		colorSensorR.fetchSample(colorDataR, 0);
		float lightStrength = colorDataR[0];
		return lightStrength;
	}
	
	/**
	 * Get the the light strength from the left color sensor
	 * 
	 * */
	public float getColorDataL(){
		colorSensorL.fetchSample(colorDataL, 0);
		float lightStrength = colorDataL[0];
		return lightStrength;
	}
	
	/**
	 * Correct the heading of the robot by using the correctionAngle calculated in the doLocalization()
	 * method
	 * 
	 * @param rightFirst Boolean which states which light sensor hit a black line first
	 * @param correctionAngle Angle that the heading must correct with
	 * 
	 * @return void
	 * */
	private void correctHeading(boolean rightFirst, double correctionAngle)
	{
		rest(500);
		
		nav.setSpeeds(SPEED,SPEED);
		Sound.twoBeeps();
		
		//If rightFirst is true, turn clockwise, else turn counterclockwise
		if(rightFirst)
		{
			this.leftMotor.rotate(convertAngle(odo.getWheelRadius(),odo.getBaseWidth(),correctionAngle),true);
			this.rightMotor.rotate(-convertAngle(odo.getWheelRadius(),odo.getBaseWidth(),correctionAngle),false);
		}
		else
		{
			this.leftMotor.rotate(-convertAngle(odo.getWheelRadius(),odo.getBaseWidth(),correctionAngle),true);
			this.rightMotor.rotate(convertAngle(odo.getWheelRadius(),odo.getBaseWidth(),correctionAngle),false);
		}

		rest(1000);
	}
	
	/**
	 * Convert distance to degrees
	 * 
	 * @param radius Radius of the robot
	 * @param distance Distance of rotation in radians
	 * 
	 * @return Converted distance in cm to degrees
	 * */
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	/**
	 * Convert Angle to degrees
	 * 
	 * @param radius Radius of the robot
	 * @param distance Distance of rotation in radians
	 * 
	 * @return Distance for wheels to turn in degrees
	 * */
	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
	
	/**
	 * Let the robot rest for millis milliseconds by stopping the motors and sleeping the thread
	 * 
	 * @param millis Milliseconds wanted for rest period
	 * 
	 * @return void
	 * */
	private void rest(int millis)
	{
		nav.setSpeeds(0,0);
		try{Thread.sleep(millis);}catch(Exception e){}
	}
	
}
