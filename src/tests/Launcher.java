/*Thomas Philippon and Olisa Okwonkwo group 30
 * This class allows the robot to launch a ball to 3 different location, in front a (0,30) on the right at (10,30) and on the left at (-10,30)
 * Take note that we commented the go forward in the navigation in order to use it to orient the robot in the right direction only using this method
 */


package tests;


import tba.Navigation;
import tba.Odometer;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.utility.Delay;
import tba.Navigation;
import tba.Odometer;

public class Launcher {
	
	private Odometer odometer;
	private Navigation nav;
	private EV3LargeRegulatedMotor leftMotor, rightMotor, launchMotor;
	//speed and acceleration of motors
	final static int SPEED = 670, ACCELERATION = 7800, SIDESPEED= 790, SIDEACCELERATION= 8800;
	//angle at which the launcher arm will rotate
	final static int LauncherRotation = 150;
	//delay time after the robot has turned and before it launches the ball
    final static int pause = 1000;
	//Constructor 
	
	public Launcher (EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, EV3LargeRegulatedMotor launchMotor, Odometer odometer, Navigation nav ){
		
		this.leftMotor= leftMotor;
		this.rightMotor= rightMotor;
		this.launchMotor= launchMotor;
		this.odometer= odometer;
		this.nav= nav;
		
	}
	
	
	public void launchRight(){
		//set speed of motors 
		launchMotor.setSpeed(SPEED);
		launchMotor.setAcceleration(ACCELERATION);
		
		//aim to the point 
		nav.travelTo(0, 90);
		
		Delay.msDelay(pause);
		
		launchMotor.rotate(LauncherRotation);
		launchMotor.rotate(-LauncherRotation);
	
		
	}
	
	public void LaunchLeft(){
		//set speed of motors
		launchMotor.setSpeed(SIDESPEED);
		launchMotor.setAcceleration(SIDEACCELERATION);
	
		//aim to the point 
		nav.travelTo(-30, 90);
		
		Delay.msDelay(pause);
		
		launchMotor.rotate(LauncherRotation);
		launchMotor.rotate(-LauncherRotation);
		
	}
	
	public void LaunchRight(){
		//set speed of motors
				launchMotor.setSpeed(SIDESPEED);
				launchMotor.setAcceleration(SIDEACCELERATION+400);
			
				//aime to the point 
				nav.travelTo(30, 90);
				
				Delay.msDelay(pause);
				
				launchMotor.rotate(LauncherRotation);
				launchMotor.rotate(-LauncherRotation);
	}
	
	
}
