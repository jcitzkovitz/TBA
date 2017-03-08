package tba;

import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;


public class LightLocalization {
	private Odometer odo;
	private SampleProvider colorSensor;
	private float[] colorData;	
	private Navigation navi;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	private double lightDistance = 11.3;
	
	public LightLocalization(Odometer odo, SampleProvider colorSensor, float[] colorData, Navigation navi) {
		this.odo = odo;
		this.colorSensor = colorSensor;
		this.colorData = colorData;
		this.navi= navi;
		EV3LargeRegulatedMotor[] motors = this.odo.getMotors();
		this.leftMotor = motors[0];
		this.rightMotor = motors[1];
	}
	
	public void doLocalization() {
		//boolean variables to check if a line was detected or not
		boolean FirstlineDetected=false;
		boolean SecondlineDetected=false;
		boolean ThirdlineDetected= false;
		boolean FourthlineDetected= false;
		
		int lineCounter=0;			//counts the number of lines detected.
		double minLight = 0.25;		//minimum light reflected by a black line
		float lightStrength;		//current light reflection recorded by the light sensor
		double thetaYminus= 0, thetaXplus=0, thetaYplus=0, thetaXminus=0, thetaY, thetaX, deltatheta, theta=0;
		double[] position = new double[3];
		
		// drive to location listed in tutorial
		navi.turnTo(90, true);
		
		
		// start rotating and clock all 4 gridlines
		// do trig to compute (0,0) and 0 degrees
		// when done travel to (0,0) and turn to 0 degrees
		
		
		
		//navi.turnTo(45, true); //turn the robot towards a heading of 45 degrees
		//navi.travelTo(-6, -3);//travel towards (-6,-3)
		//navi.turnTo(225, true); //turn the robot towards a heading of 225 degrees
		
		while(!(FirstlineDetected && SecondlineDetected && ThirdlineDetected && FourthlineDetected)){
			colorSensor.fetchSample(colorData, 0); //fetch the sample from colorSensor and store them in colorData, a buffer
			
			lightStrength = colorData[0];	
			
			
			leftMotor.setSpeed(100);
			rightMotor.setSpeed(100);
			leftMotor.backward();
			rightMotor.forward();
				
				
			if(lightStrength< minLight){ //if the amount of light reflected bt the light sensor is smaller than minLight
				lineCounter++;			 //increment the lineCounter by 1
				Sound.beep();
			
				if(lineCounter == 1){
					FirstlineDetected = true;			//the first line is detected
					thetaYminus= this.odo.getAng();
				}
				if(lineCounter==2){						//the second line is detected
					SecondlineDetected=true;
					thetaXplus= this.odo.getAng();
				}
				if(lineCounter==3){
					ThirdlineDetected=true;
					thetaYplus= this.odo.getAng();
				}
				if(lineCounter==4){
					FourthlineDetected= true;
					thetaXminus = this.odo.getAng();
				}
			}
		}
			
		thetaY= (thetaYminus-thetaYplus);
		thetaX= (thetaXplus-thetaXminus);
		
		double x=-lightDistance*Math.cos(thetaY*Math.PI/180/2) ;
		double y=-lightDistance*Math.cos(thetaX*Math.PI/180/2) ;
		
		//deltatheta= 90 + thetaY/2 - (thetaYminus-180);
		deltatheta= Math.atan(y/x)*180/Math.PI;
		theta = odo.getAng()-deltatheta;
		
		
		position[0]=x;
		position[1]=y;
		position[2]=theta;
		this.odo.setPosition(position,new boolean [] {true, true, true});
					
		
		navi.travelTo(0, 0);
		Sound.beep();
		Sound.beep();
		Sound.beep();
		navi.turnTo(270, true);
	}
	
}