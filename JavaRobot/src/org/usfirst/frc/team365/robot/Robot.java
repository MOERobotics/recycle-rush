package org.usfirst.frc.team365.robot;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.  Test
 */
public class Robot extends IterativeRobot {
	DriverStation ds;
	Joystick xbox = new Joystick(1);
	CANTalon turnLF = new CANTalon(4);
	CANTalon turnRF = new CANTalon(5);
	CANTalon turnLR = new CANTalon(6);
	CANTalon turnRR = new CANTalon(7);
	CANTalon driveLF = new CANTalon(0);
	CANTalon driveRF = new CANTalon(1);
	CANTalon driveLR = new CANTalon(2);
	CANTalon driveRR = new CANTalon(3);
	Encoder encoderLF = new Encoder(1,2);
	Encoder encoderRF = new Encoder(3,4);
	Encoder encoderLR = new Encoder(5,6);
	Encoder encoderRR = new Encoder(7,8);
	// PIDController controller  = new PIDController(.1,.001,0,encoderLF,turnLF);
	Encoder distance = new Encoder(9,10);
	final int ENCODER_MAGIC_NUMBER = 256;
	Gyro moeGyro = new Gyro(0);

	
	public Robot(){
		ds = DriverStation.getInstance();
	}
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	public void robotInit() {

	}

	public void disabledInit() {

	}

	public void disabledPeriodic() {

	}

	/**
	 * This function is run once each time the robot enters autonomous mode
	 */
	public void autonomousInit() {

	}

	/**
	 * This function is called periodically during autonomous
	 */
	public void autonomousPeriodic() {

	}

	/**
	 * This function is called once each time the robot enters tele-operated mode
	 */
	public void teleopInit(){
	}

	/**
	 * This function is called periodically during operator control
	 */
	public void teleopPeriodic() {
		crabDrive();
	}

	/**
	 * This function is called periodically during test mode
	 */
	public void testPeriodic() {
		LiveWindow.run();
	}

	void crabDrive() {
		double x = xbox.getRawAxis(4);
		double y = -xbox.getRawAxis(5);
		double angle = getAngle(x,y);
		double power = getPower(x,y);
		adjust(angle,encoderLF,turnLF);
		adjust(angle,encoderRF,turnRF);
		adjust(angle,encoderLR,turnLR);
		adjust(angle,encoderRR,turnRR);
		
		//adjustLF(angle);
		//adjustRF(angle);
		//adjustLR(angle);
		//adjustRR(angle);
		driveLF.set(power);
		driveRF.set(power);
		driveLR.set(power);
		driveRR.set(power);
		


	}

	void adjust(double newAngle,Encoder encoder,CANTalon talon) {
		double old = encoder.getRaw()*360./256.;
		if (old > 180) old = old - 360;
		else if (old < -180) old = old + 360;
		double error = Math.abs(newAngle - old);
		boolean dir = getDirection(old,newAngle);
		if (error > 180) error = 360 - error;
		double turnPower = turnPower(error,dir);
		talon.set(turnPower);

	}
	
/*	void adjustLF(double newAngle) {
		double oldLF = encoderLF.getRaw()*360./256.;
		if (oldLF > 180) oldLF = oldLF - 360;
		else if (oldLF < -180) oldLF = oldLF + 360;
		double errorLF = Math.abs(newAngle - oldLF);
		boolean dirLF = getDirection(oldLF,newAngle);
		if (errorLF > 180) errorLF = 360 - errorLF;
		double powerLF = turnPower(errorLF,dirLF);
		turnLF.set(powerLF);

	}

	void adjustRF(double newAngle) {
		double oldRF = encoderRF.getRaw()*360./256.;
		if (oldRF > 180) oldRF = oldRF - 360;
		else if (oldRF < -180) oldRF = oldRF + 360;
		double errorRF = Math.abs(newAngle - oldRF);
		boolean dirRF = getDirection(oldRF,newAngle);
		if (errorRF > 180) errorRF = 360 - errorRF;
		double powerRF = turnPower(errorRF,dirRF);
		turnRF.set(powerRF);

	}

	void adjustLR(double newAngle) {
		double oldLR = encoderLR.getRaw()*360./256.;
		if (oldLR > 180) oldLR = oldLR - 360;
		else if (oldLR < -180) oldLR = oldLR + 360;
		double errorLR = Math.abs(newAngle - oldLR);
		boolean dirLR = getDirection(oldLR,newAngle);
		if (errorLR > 180) errorLR = 360 - errorLR;
		double powerLR = turnPower(errorLR,dirLR);
		turnLR.set(powerLR);

	}

	void adjustRR(double newAngle) {
		double oldRR = encoderRR.getRaw()*360./256.;
		if (oldRR > 180) oldRR = oldRR - 360;
		else if (oldRR < -180) oldRR = oldRR + 360;
		double errorRR = Math.abs(newAngle - oldRR);
		boolean dirRR = getDirection(oldRR,newAngle);
		if (errorRR > 180) errorRR = 360 - errorRR;
		double powerRR = turnPower(errorRR,dirRR);
		turnRR.set(powerRR);

	}*/

	double turnPower(double error,boolean direction) {
		if (direction) return 0.3;
		else return -0.3;
	}

	boolean getDirection(double oldAngle,double newAngle) {
		if (newAngle - oldAngle > 180) return false;
		else if (newAngle - oldAngle < -180) return true;
		else if (newAngle >= oldAngle) return true;
		else return false;
	}



	double getAngle(double x,double y){
		double angle = Math.atan2(y, x)*180/Math.PI;
		if (angle > 180) angle = angle - 360;
		else if (angle < -180) angle = angle + 360;
		return angle;
	}

	double getPower(double x,double y){
		double power   = Math.abs(Math.sqrt(x*x+y*y));
		return limit(power);
	}

	double limit(double current){
		if(current<-1){
			return -1;
		}
		if(current>1){
			return 1;
		}
		return current;
	}

}
