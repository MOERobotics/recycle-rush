package org.usfirst.frc.team365.robot; // Jonathan Zhang
// ERIC
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first .wpilibj.CounterBase.EncodingType;
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
 * directory.  Shane, Oddity Chuckle
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

	
	int teleopLoop;
	int disableLoop;
	int autoLoop;
	
	double saveX;
	double saveY;
	
	
	boolean dirLF;
	boolean dirRF;
	boolean dirLR;
	boolean dirRR;
	
	Joystick functionBox = new Joystick(1);
	CANTalon lifterOne = new CANTalon(0);
	CANTalon lifterTwo = new CANTalon(15);
	
	Solenoid armLeft = new Solenoid(5);
	Solenoid armRight = new Solenoid(6);
	
	Encoder liftEncoder = new Encoder(14,16,true,EncodingType.k1X);
	
	
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
		
		if(teleopLoop%20 == 0){
			boolean newSet;
			
			SmartDashboard.putNumber("encoderLF", encoderLF.getRaw());
			SmartDashboard.putNumber("encoderRF", encoderRF.getRaw());
			SmartDashboard.putNumber("encoderRR", encoderRR.getRaw());
			SmartDashboard.putNumber("encoderLR", encoderLR.getRaw());
			
			SmartDashboard.putNumber("liftEncoder", liftEncoder.getRaw());
			
			controlLifter();
						
			
		}
		
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
		
		driveLF.set(power);
		driveRF.set(power);
		driveLR.set(power);
		driveRR.set(power);
		
	}
	
	
	void swerveDriveRobotCentric(){
		double x = xbox.getRawAxis(4);
		double y = -xbox.getRawAxis(5);
		double theta = limit(xbox.getX());
		
		
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

        adjust(0,encoderLF,turnLF);
        adjust(0,encoderRF,turnRF);
        adjust(0,encoderLR,turnLR);
        adjust(0,encoderRR,turnRR);

        left = limit(left);
        right = limit(right);

        driveLF.set(left);
        driveRF.set(left);
        driveLR.set(right);
        driveRR.set(right);
    }
    
    void swerveDrive(){
    	double x = xbox.getX();
    	double y = -xbox.getRawAxis(5);
    	
    	double angleLF;
    	double angleRF;
    	double angleRR;
    	double angleLR;
    	
    	double powerLF;
    	double powerRF;
    	double powerRR;
    	double powerLR;
    	
    	double twist;
    	
    	if (xbox.getX() > 0.3) twist = 0.2;
    	else if (xbox.getX() < -0.3) twist = -0.2;
    	else twist = 0;
    // compute angle for left front
    	double newYLF = y + twist;
    	double newXLF = x + twist;
    	angleLF = getAngle(newXLF,newYLF);
    	powerLF = getPower(newXLF, newYLF);
    // compute angle for right front
    	double newYRF = y - twist;
    	double newXRF = x + twist;
    	angleRF = getAngle(newXRF, newYRF);
    	powerRF = getPower(newXRF, newYRF);
    //compute angle for right rear
    	double newYRR = y - twist;
    	double newXRR = x - twist;
    	angleRR = getAngle(newXRR, newYRR);
    	powerRR = getPower(newXRR, newYRR);
    //compute angle for left rear
    	double newYLR = y + twist;
    	double newXLR = x - twist;
    	angleLR = getAngle(newXLR, newYLR);
    	powerLR =getPower(newXLR, newYLR);
    	
    	adjust(angleLF,encoderLF,turnLF);
		adjust(angleRF,encoderRF,turnRF);
		adjust(angleLR,encoderLR,turnLR);
		adjust(angleRR,encoderRR,turnRR);
    
		//double powerLF = getPower(xLF,yLF);
    }
    
    void controlLifter(){
    	
    	double power = -functionBox.getRawAxis(5);
    	power = .5*power;
    	
    	lifterOne.set(power);
    	lifterTwo.set(power);
    	
    	if(functionBox.getRawButton(1)){
    			armLeft.set(false);
    			armRight.set(false);
    	}else if(functionBox.getRawButton(2)) {
    			armLeft.set(false);
    			armRight.set(true);
    			
    	}else if(functionBox.getRawButton(3)){
    			armLeft.set(true);
    			armRight.set(false);
    			
    	}else if(functionBox.getRawButton(4)){
    			armLeft.set(true);
    			armRight.set(true);
    	}
    	
    		
    		
    }
    

}
