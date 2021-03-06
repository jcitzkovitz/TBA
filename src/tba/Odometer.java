/*
 * File: Odometer.java
 * Written by: Sean Lawlor
 * ECSE 211 - Design Principles and Methods, Head TA
 * Fall 2011
 * Ported to EV3 by: Francois Ouellet Delorme
 * Fall 2015
 * 
 * Class which controls the odometer for the robot
 * 
 * Odometer defines cooridinate system as such...
 * 
 * 					90Deg:pos y-axis
 * 							|
 * 							|
 * 							|
 * 							|
 * 180Deg:neg x-axis------------------0Deg:pos x-axis
 * 							|
 * 							|
 * 							|
 * 							|
 * 					270Deg:neg y-axis
 * 
 * The odometer is initalized to 90 degrees, assuming the robot is facing up the positive y-axis
 * 
 */
package tba;
import lejos.utility.Timer;
import lejos.utility.TimerListener;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * The Odometer class keeps track of the position and angle that the robot has
 * moved. It uses the information given by the wheel motor's sensors, which tracks
 * how many revolutions have been made, and produces other valuable information
 * needed for the robot's functionality.*/

public class Odometer implements TimerListener {

	private Timer timer;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	private final int DEFAULT_TIMEOUT_PERIOD = 20;
	private double leftRadius, rightRadius, width;
	private double x, y, theta;
	private double[] oldDH, dDH;
	
	/**
	 * Odometer constructer
	 * 
	 * @param 	leftMotor 	Left wheel motor
	 * @param 	rightMotor 	Right wheel motor
	 * @param 	INTERVAL 	Interval of time between polling
	 * @param 	autoStart 	States whether to start thread automatically or not
	 * 
	 * */
	public Odometer (EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, int INTERVAL, boolean autostart) {
		
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		
		// default values, modify for your robot
		this.rightRadius = 2.03;
		this.leftRadius = 2.03;
		this.width = 12.8;
		
		this.x = 0.0;
		this.y = 0.0;
		this.theta = 90.0;
		this.oldDH = new double[2];
		this.dDH = new double[2];

		if (autostart) {
			// if the timeout interval is given as <= 0, default to 20ms timeout 
			this.timer = new Timer((INTERVAL <= 0) ? INTERVAL : DEFAULT_TIMEOUT_PERIOD, this);
			this.timer.start();
		} else
			this.timer = null;
	}
	
	/**
	 * Functions to stop the timerlistener
	 * 
	 * @return 		void
	 * */
	public void stop() {
		if (this.timer != null)
			this.timer.stop();
	}
	
	/**
	 * Functions to start the timerlistener
	 * 
	 * @return 		void
	 * */
	public void start() {
		if (this.timer != null)
			this.timer.start();
	}
	
	/**
	 * Calculates displacement and heading as title suggests
	 * 
	 * @param 	data 	Holds data values from tachometers
	 * 
	 * @return 		void
	 * */
	private void getDisplacementAndHeading(double[] data) {
		int leftTacho, rightTacho;
		leftTacho = leftMotor.getTachoCount();
		rightTacho = rightMotor.getTachoCount();

		data[0] = (leftTacho * leftRadius + rightTacho * rightRadius) * Math.PI / 360.0;
		data[1] = (rightTacho * rightRadius - leftTacho * leftRadius) / width;
	}
	
	/**
	 * Recompute the odometer values using the displacement and heading changes
	 * 
	 * @return 		void
	 * */
	public void timedOut() {
		this.getDisplacementAndHeading(dDH);
		dDH[0] -= oldDH[0];
		dDH[1] -= oldDH[1];

		// update the position in a critical region
		synchronized (this) {
			theta += dDH[1];
			theta = fixDegAngle(theta);

			x += dDH[0] * Math.cos(Math.toRadians(theta));
			y += dDH[0] * Math.sin(Math.toRadians(theta));
		}

		oldDH[0] += dDH[0];
		oldDH[1] += dDH[1];
	}

	/**
	 * Return x value
	 * 
	 * @return 		x value
	 * */
	public double getX() {
		synchronized (this) {
			return x;
		}
	}

	/**
	 * Return y value
	 * 
	 * @return 		y value
	 * */
	public double getY() {
		synchronized (this) {
			return y;
		}
	}

	/**
	 * Return theta value
	 * 
	 * @return 		theta value
	 * */
	public double getAng() {
		synchronized (this) {
			return theta;
		}
	}

	/**
	 * Set x, y and theta values
	 * 
	 * @param 	position 	Array of doubles holding x, y, and theta values
	 * @param 	update	States whether the values should be updated in the odometer
	 * 
	 * @return		void
	 * */
	public void setPosition(double[] position, boolean[] update) {
		synchronized (this) {
			if (update[0])
				x = position[0];
			if (update[1])
				y = position[1];
			if (update[2])
				theta = position[2];
		}
	}
	
	/**
	 * Set x value
	 * 
	 * @param x 	X value
	 * 
	 * @return 		void
	 * */
	public void setX(double x) {
		synchronized (this) {
			this.x = x;
		}
	}
	
	/**
	 * Set y value
	 * 
	 * @param 	y 	Y value
	 * 
	 * @return		void
	 * */
	public void setY(double y) {
		synchronized (this) {
			this.y = y;
		}
	}

	/**
	 * Gets the robot's position
	 * 
	 * @param 	position 	Array of doubles containing the robot's x, y, and theta values
	 * 
	 * @return 		void
	 * */
	public void getPosition(double[] position) {
		synchronized (this) {
			position[0] = x;
			position[1] = y;
			position[2] = theta;
		}
	}

	/**
	 * Returns the robot's position
	 * 
	 * @return 		Array containing the robot's x, y, and theta values
	 * */
	public double[] getPosition() {
		synchronized (this) {
			return new double[] { x, y, theta };
		}
	}
	
	/**
	 * Return the robot's wheel radius
	 * 
	 * @return 		The wheel radius
	 * */
	public double getWheelRadius()
	{
		return this.rightRadius;
	}
	
	/**
	 * Return the robot's base width
	 * 
	 * @return 		The robot's base width
	 * */
	public double getBaseWidth()
	{
		return this.width;
	}
		
	/**
	 * Access both the left and right motors
	 * 
	 * @return 		Array containing both motors
	 * */
	public EV3LargeRegulatedMotor [] getMotors() {
		return new EV3LargeRegulatedMotor[] {this.leftMotor, this.rightMotor};
	}
	
	/**
	 * Access the left motor
	 * 
	 * @return 		The left motor
	 * */
	public EV3LargeRegulatedMotor getLeftMotor() {
		return this.leftMotor;
	}
	
	/**
	 * Access the right motor
	 * 
	 * @return 		The right motor
	 * */
	public EV3LargeRegulatedMotor getRightMotor() {
		return this.rightMotor;
	}

	/**
	 * Fix the degree angle to wrap around to 0
	 * 
	 * @return 		The orientation angle from 0-360 degrees
	 * */
	public static double fixDegAngle(double angle) {
		if (angle < 0.0)
			angle = 360.0 + (angle % 360.0);

		return angle % 360.0;
	}

	/**
	 * Set the robot's base width for different scenarios
	 * 
	 * @param 	width 	Wanted base width value
	 * 
	 * @return 		void
	 * */
	public void setBaseWidth(double width)
	{
		this.width = width;
	}
	
	/**
	 * Minimum angle calculation
	 * 
	 * @param 	a 	Point a
	 * @param 	b 	Point b
	 * 
	 * @return 		The minimum angle
	 * */
	public static double minimumAngleFromTo(double a, double b) {
		double d = fixDegAngle(b - a);

		if (d < 180.0)
			return d;
		else
			return d - 360.0;
	}
	
	
}

