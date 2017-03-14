/*Thomas Philippon and Olisa Okonkwo group 30
 * Localization with the light sensor detecting 
 * black lines while turning 
 * 
 */

package tba;

import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class LightLocalizer {
	private Odometer odo;
	private SampleProvider colorSensor;
	private float[] colorData;	
	private Navigation nav;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	//Max light value of reading for a light
	private static final int  Line_light = 40;

	//Value of reflecting light on the ground 
	private static float color; 

	//Count of the number of line since we now the robot will cross 4 
	private static int LineCount= 0;
	//Distance from the center of rotation to the light sensor
		private static final double sensorDist= 15.2;
		
		final static int  SPEED = 100;
		
		//Array containing angle value at each line from left to right
		private double[] line_Angle = new double[4]; 
		private boolean navigate = true;
		
		
	public LightLocalizer(Odometer odo,Navigation nav, float[] colorData,SampleProvider colorSensor, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor) {
		this.odo = odo;
		this.colorSensor = colorSensor;
	    this.colorData = colorData;
		this.nav=nav;
		this.leftMotor=leftMotor;
		this.rightMotor= rightMotor;
	}
	
	public void doLocalization() {
		
		
		
		// drive to location listed in tutorial
		nav.turnTo(45, true);
			leftMotor.setSpeed(SPEED);
	     	rightMotor.setSpeed(SPEED);
		 
	     	//go forward and check for a black line 
	     	while  (navigate){
	    	
	     	colorSensor.fetchSample(colorData, 0);
			 color = (colorData[0]*100);
	        
			    leftMotor.forward();
	    		rightMotor.forward();
			 
	    		//if the sensor detects a black line 
	    		if (color< Line_light){
	    			
	    			//backward of 10 cm 
	    			leftMotor.rotate(-290,true);
			        rightMotor.rotate(-290, false);
			        //stop robot 
			        navigate = false;
	    		}
	     	}
			 
		
		
		//reset the x and y position 
		odo.setPosition(new double [] {0, 0, odo.getAng()}, new boolean [] {true, true, true});
		
		// set the speed and start rotating counter clockwise
	
		leftMotor.backward();
		rightMotor.forward();
		
        	//since we know there are 4 lines to intersect, we use a while loop with an incrementing variable 
			while (LineCount<4){
				
				//initiate the sensor to retrieve the values 
				colorSensor.fetchSample(colorData, 0);
				 color = (colorData[0]*100);
				 
				 if (color< Line_light){
					 //if a line is intersected, play sound and record the value in the array
					 Sound.playNote(Sound.FLUTE, 440, 250);
					 line_Angle[LineCount++] = odo.getAng();
					 
					 //delay to avoid reading to values at the same line 
					Delay.msDelay(2000);
			}
		
		}
		        // stop motors when the fourth line is intersected 
				leftMotor.stop(true);
		        rightMotor.stop(false);
		
		        // Compute  x, y and theta differences
		        // using formulas from the tutorial slides 
		        double   angleX = line_Angle[1] - line_Angle[3];
				double angleY = line_Angle[0] - line_Angle[2];
				
				double lengthX =(-1)* sensorDist * Math.cos(Math.toRadians(angleY/2));
				double lengthY = (-1)*sensorDist * Math.cos(Math.toRadians(angleX/2));
				
				double deltaTheta = -(angleY/2) +(line_Angle[3]-180)+90;
				 
				// set the new position in odometer
				odo.setPosition(new double [] {(lengthX), (lengthY), deltaTheta}, new boolean [] {true, true, true});
				Sound.beep();
				
				// At the end turn and travel to (0,0)
				nav.travelTo(0.0, 0.0);
				nav.turnTo(0.0,true);	
				
				// set the angle to zero and axis to the o as well
				odo.setPosition(new double [] {0.0, 0.0, 0}, new boolean [] {true, true, true});
			}


	
	

}
