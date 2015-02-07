package org.usfirst.frc.team365.robot;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.  Test
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
	Encoder encoderLF = new Encoder(4,5,true,EncodingType.k1X);
	Encoder encoderRF = new Encoder(0,1,true,EncodingType.k1X);
	Encoder encoderLR = new Encoder(6,7,true,EncodingType.k1X);
	Encoder encoderRR = new Encoder(8,9,true,EncodingType.k1X);
	
	
	 //PIDController controller  = new PIDController(.1,.001,0,encoderLF,turnLF);
	//Encoder distance = new Encoder(9,10);
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
		if(xbox.getZ()>.5){
			this.encoderLF.reset();
			encoderRF.reset();
			encoderLR.reset();
			encoderRR.reset();
		}
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
		this.encoderLF.reset();
		this.encoderRF.reset();
		this.encoderLR.reset();
		this.encoderRR.reset();
	}

	/**
	 * This function is called periodically during operator control
	 */
	public void teleopPeriodic() {
		if(xbox.getZ()>.5){
			this.moeGyro.reset();
			this.encoderLF.reset();
			encoderRF.reset();
			encoderLR.reset();
			encoderRR.reset();
		}
		this.swerveDriveRobotCentric();
	}

	/**
	 * This function is called periodically during test mode
	 */
	public void testPeriodic() {
	
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
		
		driveLF.set(power);
		driveRF.set(power);
		driveLR.set(power);
		driveRR.set(power);
		
	}
	
	
	void swerveDriveRobotCentric(){
		double x = xbox.getRawAxis(4);
		if(Math.abs(x)<.1){
			x=0;
		}
		double y = -xbox.getRawAxis(5);
		if(Math.abs(y)<.1){
			y=0;
		}
		double theta =xbox.getX();
		if(Math.abs(theta)<.1){
			theta=0;
		}
		
		
		double xLF = x +theta;
		xLF = -xLF;
		double yLF = y +theta;
		double angleLF = getAngle(xLF,yLF);
		double powerLF = getPower(xLF,yLF);
		
		double xRF = x +theta;
		double yRF = y -theta;
		yRF=-yRF;
		double angleRF = getAngle(xRF,yRF);
		double powerRF = getPower(xRF,yRF);
		
		double xLR = x -theta;
		xLR =-xLR;
		double yLR = y +theta;
		
		double angleLR = getAngle(xLR,yLR);
		double powerLR = getPower(xLR,yLR);
		
		double xRR = x -theta;
		double yRR = y -theta;
		yRR=-yRR;
		double angleRR = getAngle(xRR,yRR);
		double powerRR = getPower(xRR,yRR);
		
		
		
		adjust(angleLF,encoderLF,turnLF);
		adjust(angleRF,encoderRF,turnRF);
		adjust(angleLR,encoderLR,turnLR);
		adjust(angleRR,encoderRR,turnRR);
		
		
		//deadband
		if(Math.abs(powerLF)>.15 || Math.abs(powerRF)>.15 || Math.abs(powerLR)>.15 || Math.abs(powerRR)>.15 ){		
		driveLF.set(powerLF);
		driveRF.set(powerRF);
		driveLR.set(powerLR);
		driveRR.set(powerRR);
		}else{
			driveLF.set(0);
			driveRF.set(0);
			driveLR.set(0);
			driveRR.set(0);	
		}
		SmartDashboard.putNumber("encoderLF", encoderLF.getRaw()*360/this.ENCODER_MAGIC_NUMBER);
		SmartDashboard.putNumber("encoderRF", encoderRF.getRaw()*360/this.ENCODER_MAGIC_NUMBER);
		SmartDashboard.putNumber("encoderLR", encoderLR.getRaw()*360/this.ENCODER_MAGIC_NUMBER);
		SmartDashboard.putNumber("encoderRR", encoderRR.getRaw()*360/this.ENCODER_MAGIC_NUMBER);
		
	}
	

	
	double error(double angle, double target){
		double error = target-angle;
		if(error<0){
			error = target+360-angle;
		}
		return error;
	}

	void adjust(double target,Encoder encoder,CANTalon talon) {
		double angle = norm(encoder.getRaw()*360./256.);
		
			
			target = norm(target);
			double targetinv = norm(target-180);
			//if(Math.min(error(angle,target), error(target,angle))>Math.min(error(angle,targetinv), error(targetinv,angle))){
			//	target = targetinv;
			//}
			if(error(angle,target)<error(target,angle)){
				talon.set(turnPower(error(angle,target)));
			}else{
				talon.set(turnPower(-error(target,angle)));
			}
		
	}
	




	double norm(double degree){
		double result = Math.round(degree)%360;
		if(result<0){
			result=360+result;
		}
		return result;
	}

	double getAngle(double x,double y){
		double angle = atan2mod(y, x)-90;
	
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
	
	double sign(double input){
		return input/Math.abs(input);
	}
	
	double atan2mod(double y, double x){
		double result = Math.atan2(y,x)*180/Math.PI;
		if(result<0){
			result= 360+result;
		}
		return result;
	}
	
	double turnPower(double input){
		if(Math.abs(input)>3){
			if(Math.abs(input)>20){
				 return sign(input)*.3;
			}
			return sign(input)*.1;
		}
		return 0;
	}

}
