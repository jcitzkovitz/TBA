package tba;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;


public class LightLocalizerV4 {

	private Odometer odo;
	private SampleProvider colorSensorR;
	private float[] colorDataR;	
	private SampleProvider colorSensorL;
	private float[] colorDataL;	
	private Navigation nav;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	private float SPEED = 100;
	private double lightSensorDistance = 2.1;
	
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
	
	public void doLocalization() {
		
		double minLight =0.3;
		
		// Travel in the x direction until a black line is seen by both sensors
		boolean rightHit = false,leftHit = false;
		double firstHit = 0;
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
		
		double currentY = odo.getY();
		while(Math.abs(currentY-odo.getY()) < lightSensorDistance)
		{
			nav.setSpeeds(SPEED,SPEED);
		}
		
		nav.setSpeeds(0,0);
		try{Thread.sleep(500);}catch(Exception e){}
		
		// Turn to positive x direction and travel until both sensors sense a black line and perform correction
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
			
			double currentX = odo.getX();
			while(Math.abs(currentX-odo.getX()) < lightSensorDistance)
			{
				nav.setSpeeds(SPEED,SPEED);
			}
			nav.setSpeeds(0,0);
			try{Thread.sleep(500);}catch(Exception e){}
			
	}
	
	public float getColorDataR(){
		colorSensorR.fetchSample(colorDataR, 0);
		float lightStrength = colorDataR[0];
		return lightStrength;
	}
	
	public float getColorDataL(){
		colorSensorL.fetchSample(colorDataL, 0);
		float lightStrength = colorDataL[0];
		return lightStrength;
	}
	
	private void correctHeading(boolean rightFirst, double correctionAngle)
	{
		nav.setSpeeds(0,0);
		try{Thread.sleep(500);}catch(Exception e){}
		nav.setSpeeds(SPEED,SPEED);
		Sound.twoBeeps();
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
		nav.setSpeeds(0, 0);
		try{Thread.sleep(1000);}catch(Exception e){}
	}
	
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
	
}
