/*Thomas Philippon 
 * test 1
 * Mars 1st 12 AM: The purpose of this class is to test the efficiency of the different motors. The class will make the robot run in a certain path and retrieving the final position
 * reported by the odometer and the real position measured, we will be able to determine which two motors are the more precise
 */


package tests;

public class TestRun {

	private Navigation navigation;
	//speed and acceleration of motors
	final static int SPEED = 300;


	//Constructor 
	public TestRun ( Navigation navigation ){
		
		this.navigation= navigation;
	}
	
	
	//method to test the robot position after a straight line 
	public void StraightTest (){
		 
		navigation.goForward(91.44);
		
	}
	
	//Method to test the robot after traveling a square
	public void TurnTest(){
		navigation.travelTo(0,60.96);
		navigation.travelTo(60.96, 60.96);
		navigation.travelTo(60.96, 0);
		navigation.travelTo(0, 0);
	}
	
	
}
	
	