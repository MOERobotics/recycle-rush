package org.usfirst.frc.team365.robot;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	DriverStation ds;
	Joystick xbox = new Joystick(0);
	CANTalon turnLF = new CANTalon(3);
	CANTalon turnRF = new CANTalon(4);
	CANTalon turnLR = new CANTalon(12);
	CANTalon turnRR = new CANTalon(11);
	
	CANTalon driveLF = new CANTalon(1);
	CANTalon driveRF = new CANTalon(2);
	CANTalon driveLR = new CANTalon(14);
	CANTalon driveRR = new CANTalon(13);
	
	CANTalon lifterOne = new CANTalon(0);
	CANTalon lifterTwo = new CANTalon(15);
	
	Solenoid canGrab = new Solenoid(0);
	Solenoid hook = new Solenoid(1);
	
	
	
	
	Encoder encoderLF = new Encoder(4,5,true,EncodingType.k1X);
	Encoder encoderRF = new Encoder(0,1,true,EncodingType.k1X);
	Encoder encoderLR = new Encoder(6,7,true,EncodingType.k1X);
	Encoder encoderRR = new Encoder(8,9,true,EncodingType.k1X);
//	PIDController controller  = new PIDController(.1,.001,0,encoderLF,turnLF);
	
	Encoder distEncoder = new Encoder(2,30,true,EncodingType.k1X);
	final int ENCODER_MAGIC_NUMBER = 256;
	
	Gyro moeGyro = new Gyro(0);
	
	int teleopLoop;
	int disableLoop;
	int autoLoop;
	
	double saveX;
	double saveY;
	
	
	boolean dirLF;
	boolean dirRF;
	boolean dirLR;
	boolean dirRR;
	
	public Robot(){
		ds = DriverStation.getInstance();
		
		
	}
	
	
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
    	turnLF.enableBrakeMode(true);
    	turnRF.enableBrakeMode(true);
    	turnLR.enableBrakeMode(true);
    	turnRR.enableBrakeMode(true);
    	
    	moeGyro.setSensitivity(.0125);
    	
    	
    }
    
    public void disabledInit() {
    	disableLoop = 0;
    }
    
    public void disabledPeriodic() {
    	disableLoop++;
    }
    
    /**
     * This function is run once each time the robot enters autonomous mode
     */		
    public void autonomousInit() {
    	autoLoop = 0;
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
    	autoLoop++;
    }
    
    /**
     * This function is called once each time the robot enters tele-operated mode
     */
    public void teleopInit(){
    	teleopLoop = 0;
    	saveX = 0;
    	saveY = 0.7;
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
    	teleopLoop++;
    	//testRoutine();
    	crabDrive();
    	
    	if (teleopLoop % 10 == 0){
    		
    		SmartDashboard.putNumber("encoderLF", encoderLF.getRaw());
    		SmartDashboard.putNumber("encoderRF", encoderRF.getRaw());
    		SmartDashboard.putNumber("encoderLR", encoderLR.getRaw());
    		SmartDashboard.putNumber("encoderRR", encoderRR.getRaw());
    		
    		SmartDashboard.putNumber("distance", distEncoder.getRaw());
    	}
    	
    	if (teleopLoop % 50 == 0 ) {
    		
    	}
    	canGrab.set(true);
    	hook.set(false);
    	
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
    	double power = getPower(x,y);
    	if (power <= 0.3){
    		x = saveX;
    		y = saveY;
    	}
    	
    	else{
    		power = (power - 0.3)/0.7;
    	}
    	
    	double angle = getAngle(x,y);
    	
    	saveX = x;
    	saveY = y;
    	
    	
    	adjustLF(angle);
    	adjustRF(angle);
    	adjustLR(angle);
    	adjustRR(angle);
    	driveLF.set(power);
    	driveRF.set(power);
    	driveLR.set(power);
    	driveRR.set(power);
    }
    
    void adjustLF(double newAngle) {
    	double oldLF = encoderLF.getRaw()*360./256.;
    	if (oldLF > 180) oldLF = oldLF - 360;
    	else if (oldLF < -180) oldLF = oldLF + 360;
    	double errorLF = Math.abs(newAngle - oldLF);
    	dirLF = getDirection(oldLF,newAngle);
    	if (errorLF > 180) errorLF = 360 - errorLF;
    	double powerLF = turnPower(errorLF,dirLF);
    	turnLF.set(powerLF);
    	
    }
    
    void adjustRF(double newAngle) {
    	double oldRF = encoderRF.getRaw()*360./256.;
    	if (oldRF > 180) oldRF = oldRF - 360;
    	else if (oldRF < -180) oldRF = oldRF + 360;
    	double errorRF = Math.abs(newAngle - oldRF);
    	dirRF = getDirection(oldRF,newAngle);
    	if (errorRF > 180) errorRF = 360 - errorRF;
    	double powerRF = turnPower(errorRF,dirRF);
    	turnRF.set(powerRF);
    	
    }
    
    void adjustLR(double newAngle) {
    	double oldLR = encoderLR.getRaw()*360./256.;
    	if (oldLR > 180) oldLR = oldLR - 360;
    	else if (oldLR < -180) oldLR = oldLR + 360;
    	double errorLR = Math.abs(newAngle - oldLR);
    	dirLR = getDirection(oldLR,newAngle);
    	if (errorLR > 180) errorLR = 360 - errorLR;
    	double powerLR = turnPower(errorLR,dirLR);
    	turnLR.set(powerLR);
    	
    }
    
    void adjustRR(double newAngle) {
    	double oldRR = encoderRR.getRaw()*360./256.;
    	if (oldRR > 180) oldRR = oldRR - 360;
    	else if (oldRR < -180) oldRR = oldRR + 360;
    	double errorRR = Math.abs(newAngle - oldRR);
    	dirRR = getDirection(oldRR,newAngle);
    	if (errorRR > 180) errorRR = 360 - errorRR;
    	double powerRR = turnPower(errorRR,dirRR);
    	turnRR.set(powerRR);
    	
    }
    
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
    
    void testRoutine(){
    	double x = xbox.getX();
    	double y = -xbox.getRawAxis(5);
    	if (x < 0.1 && x > -0.1) x = 0;
    	if (y < 0.1 && y > -0.1) y = 0;
    	
    	turnLF.set(x);
    	driveLF.set(y);
    	
    	turnLF.set(x);
    	turnRF.set(x);
    	turnLR.set(x);
    	turnRR.set(x);
    	driveLF.set(y);
    	driveRF.set(y);
    	driveLR.set(y);
    	driveRR.set(y);
    	
    }
    
    void tankDrive(){
    	double x = xbox.getRawAxis(4);
    	double y = - xbox.getRawAxis(5);
    	
    	if (x < 0.1 && x > -0.1) x = 0;
    	if (y < 0.1 && y > -0.1) y = 0;
    	
    	double left = y + x;
    	double right = y - x;
    	
    	adjustLF(0);
    	adjustRF(0);
    	adjustLR(0);
    	adjustRR(0);
    	
    	left = limit(left);
    	right = limit(right);
    	
    	driveLF.set(left);
    	driveRF.set(left);
    	driveLR.set(right);
    	driveRR.set(right);
    }

    
}
