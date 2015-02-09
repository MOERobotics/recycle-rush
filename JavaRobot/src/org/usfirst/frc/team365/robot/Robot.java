package org.usfirst.frc.team365.robot;

//@SurgicalPrecision MOE365
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Gyro;
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
	CANTalon twistLeftFront = new CANTalon(3);
	CANTalon twistRightFront = new CANTalon(4);
	CANTalon twistLeftRear = new CANTalon(12);
	CANTalon twistRightRear = new CANTalon(11);
	
	CANTalon driverLeftFront = new CANTalon(1);
	CANTalon driverRightFront = new CANTalon(2);
	CANTalon driverLeftRear = new CANTalon(14);
	CANTalon driverRightRear = new CANTalon(13);
	
	CANTalon lifterOne = new CANTalon(0);
	CANTalon lifterTwo = new CANTalon(15);
	
	Encoder encoderLF = new Encoder(4,5,true,EncodingType.k1X);
	Encoder encoderRF = new Encoder(0,1,true,EncodingType.k1X);
	Encoder encoderLR = new Encoder(6,7,true,EncodingType.k1X);
	Encoder encoderRR = new Encoder(8,9,true,EncodingType.k1X);
	
	Encoder distEncoder = new Encoder(2,3,true,EncodingType.k1X);
	
	Gyro moeGyro = new Gyro(1);
	
	Joystick xBox = new Joystick(0);
	
	int autoLoop;
	int autoStep;
	int disabledLoop;
	int teleopLoop;
	boolean lastDirLF;
	boolean lastDirRF;
	boolean lastDirLR;
	boolean lastDirRR;
	double deltaLF;
	double deltaRF;
	double deltaLR;
	double deltaRR;
	double sumLF;
	double sumRF;
	double sumLR;
	double sumRR;
	int countTime;
	boolean overshootLF;
	boolean overshootRF;
	boolean overshootLR;
	boolean overshootRR;
	double setPoint;
	boolean setLF;
	boolean setRF;
	boolean setLR;
	boolean setRR;
	boolean nearLF;
	boolean nearRF;
	boolean nearLR;
	boolean nearRR;
	double maxCurLF;
	double maxCurRF;
	double maxCurLR;
	double maxCurRR;
	double saveX;
	double saveY;
	double straightSum;
	boolean last1;
	boolean last2;
	boolean last3;
	boolean last4;
	boolean last5;
	boolean last7;
	boolean last8;
	double lastAxis3;
	double lastPad6;
	boolean dirLF;
	boolean dirRF;
	boolean dirLR;
	boolean dirRR;
	boolean reverse;
	boolean allSet;
	boolean setWheels;
	boolean swerveMode;
	double fieldX;
	double fieldY;
	
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
	
	public Robot() {
		ds = DriverStation.getInstance();
	}
    public void robotInit() {
    	
    	twistLeftFront.enableBrakeMode(true);
    	twistRightFront.enableBrakeMode(true);
    	twistLeftRear.enableBrakeMode(true);
    	twistRightRear.enableBrakeMode(true);
    	
    	moeGyro.setSensitivity(.0125);
    	
    	saveX = 0;
    	saveY = 0.8;
    	swerveMode = true;
    	
    	
    	
    }
    
    public void disabledInit()  {
    	disabledLoop = 0;
    }
    
    public void disabledPeriodic() {
    	disabledLoop++;
    	if (xBox.getRawButton(8))  {
    		encoderLF.reset();
    		encoderRF.reset();
    		encoderLR.reset();
    		encoderRR.reset();
    		
    		saveX = 0;
    		saveY = 0.8;
    		
    		distEncoder.reset();
    		
    	}
    	
    	if (xBox.getRawButton(7)) {
    		moeGyro.reset();
    	}
    	
    	if (disabledLoop % 20 == 0) {
    		
    		int wheelLF = encoderLF.getRaw();
    		int wheelRF = encoderRF.getRaw();
    		int wheelLR = encoderLR.getRaw();
    		int wheelRR = encoderRR.getRaw();
    		
    		int dist = distEncoder.getRaw();
    		
    		double bearing = moeGyro.getAngle();
    		
    		SmartDashboard.putNumber("encoderLF", wheelLF);
    		SmartDashboard.putNumber("encoderRF", wheelRF);
    		SmartDashboard.putNumber("encoderLR", wheelLR);
    		SmartDashboard.putNumber("encoderRR", wheelRR);
    		
    		SmartDashboard.putNumber("distance", dist);
    		
    		SmartDashboard.putNumber("gyro", bearing);
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
    	if(autoLoop < 100) //Check if we've completed 100 loops (approximately 2 seconds)
		{
			
			autoLoop++;
		}
		
    }
    
    /**
     * This function is called once each time the robot enters tele-operated mode
     */
    public void teleopInit(){
    	teleopLoop = 0;
    	lastPad6 = 0;
    	lastAxis3 = 0;
    	last1 = false;
    	last2 = false;
    	last3 = false;
    	last4 = false;
    	last5 = false;
    	last7 = false;
    	last8 = false;
    	nearLF = false;
    	nearRF = false;
    	nearLR = false;
    	nearRR = false;
    	setLF = false;
    	setRF = false;
    	setLR = false;
    	setRR = false;
    	swerveMode = true;
    	setWheels = false;
    	allSet = false;
 //   	saveX = 0.;
//    	saveY = 0.7;
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
    	
   // 	setPoint = SmartDashboard.getNumber("DB/Slider 0", 0.0);
    	
    	setPoint = 3;
    	
    	boolean new1 = xBox.getRawButton(1);
    	boolean new2 = xBox.getRawButton(2);
    	boolean new3 = xBox.getRawButton(3);
    	boolean new4 = xBox.getRawButton(4);
    	boolean new5 = xBox.getRawButton(5);
    	boolean new7 = xBox.getRawButton(7);
    	boolean new8 = xBox.getRawButton(8);
    	
    	double newAxis3 = xBox.getRawAxis(3);
    	double newPad6 = xBox.getRawAxis(6);
    	
    	teleopLoop++;
    	double robotBearing = moeGyro.getAngle();
    	if (robotBearing >=360 || robotBearing <=-360) moeGyro.reset();
    	
    	encoderRevolutionCheck();
    	
 //   	testRoutine();
    	wheelAngleTestRoutine();
    	
/*
    	if (newPad6 > -0.5) {
    		if (lastPad6 > -0.5 && lastPad6 < 0.5) {
    		//	allSet = false;
    			startWheelSet();
    			// 			setWheels = true;
    		}
    		turnInPlace();
    		saveX = 0;
    		saveY = 0.7;
    	}
    	else if (new7 || new8) {
    		if (!last7 && !last8) {
    		//	allSet = false;
    			startWheelSet();
    			//		  setWheels = true;
    		}
    		gentleStrafe();
    		saveX = 0.7;
    		saveY = 0;
    	}
    	else if (!swerveMode) {
    		// 	  carMode();
    		tankDrive();
    	}
    	else {
    		crabDrive360();
    		//   	  swerveDrive360();
    	}

    	if (new1 && !last1) {
    		swerveMode = false;
    	//	allSet = false;
    		startWheelSet();
    	}
    	else if (new4 && !last4) swerveMode = true;
    	/*
    	else if (new3 && !last3) {
    		saveX = 0;
    		saveY = 0.7;
    		startWheelSet();
    	}
    	else if (new2 && !last2) {
    		saveX = 0.7;
    		saveY = 0;
    		startWheelSet();
    	}*/

    	lastPad6 = newPad6;
    	last1 = new1;
    	last2 = new2;
    	last3 = new3;
    	last4 = new4;
    	last5 = new5;
    	last7 = new7;
    	last8 = new8;
    	lastAxis3 = newAxis3;

    	if (teleopLoop % 20 == 0) {

    		boolean newSet;

    		int wheelLF = encoderLF.getRaw();
    		int wheelRF = encoderRF.getRaw();
    		int wheelLR = encoderLR.getRaw();
    		int wheelRR = encoderRR.getRaw();

    		int dist = distEncoder.getRaw();

    		double bearing = moeGyro.getAngle();

    		SmartDashboard.putNumber("encoderLF", wheelLF);
    		SmartDashboard.putNumber("encoderRF", wheelRF);
    		SmartDashboard.putNumber("encoderLR", wheelLR);
    		SmartDashboard.putNumber("encoderRR", wheelRR);

    		SmartDashboard.putNumber("distance", dist);

    		SmartDashboard.putNumber("gyro", bearing);
    		if (setLF && setRF && setLR && setRR) newSet = true;
    		else newSet = false;
    		SmartDashboard.putBoolean("wheelsSet", newSet);
    	}
    }

    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    	LiveWindow.run();
    }
    
    void crabDrive360() {
    	double power;
    	double x = xBox.getRawAxis(4);
    	double y = -xBox.getRawAxis(5);

 //   	if (Math.abs(y) > Math.abs(x)) power = Math.abs(y);
 //   	else power = Math.abs(x);
    	
    	power = Math.sqrt(x*x + y*y);
    	power = limit(power);

    	if (power <= 0.3) {
 //   	if (x<0.4 && x>-0.4 && y<0.4 && y>-0.4) {
    		x = saveX;
    		y = saveY;	
    		power = 0;
    		if (!setWheels) startWheelSet();
   // 		setWheels = true;
    	}
    	else {
    		power = (power - 0.3)/0.7;
    		setWheels = false;
    	}
    	
    	if (xBox.getRawButton(5)) power = 0.5*power;

    	saveX = x;
    	saveY = y;

    	double wheelAngle = computeAngle(y,x);
 /*   	             //                            Field Centric Code
    	double bearing = moeGyro.getAngle();
    	if (bearing > 180) bearing = bearing - 360;
    	else if (bearing < -180) bearing = bearing + 360;
    	wheelAngle = wheelAngle - bearing;
    	if (wheelAngle > 180) wheelAngle = wheelAngle - 360;
    	else if (wheelAngle < -180) wheelAngle = wheelAngle + 360;
*/
    	adjustAllWheels(wheelAngle,wheelAngle,wheelAngle,wheelAngle);

    	if (!nearLF || !nearRF || !nearLR || !nearRR) power = 0;

    	driveAllWheels(power,power,power,power);

    }
    
    void swerveDrive360() {
    	double power;
    	double angleLF;
    	double angleRF;
    	double angleLR;
    	double angleRR;
    	
    	double xNew;
    	double yNew;
    	double x = xBox.getRawAxis(4);
    	double y = -xBox.getRawAxis(5);
    	
//    	if (Math.abs(y) > Math.abs(x)) power = Math.abs(y);
//    	else power = Math.abs(x);
    	
    	double twist = xBox.getX();
    	if (twist > 0.5) twist = 0.15;
    	else if (twist < -0.5) twist = -0.15;
    	else twist = 0;

    	power = Math.sqrt(x*x + y*y);
    	power = limit(power);

    	if (power <=0.3) {
  //  	if (x<0.4 && x>-0.4 && y<0.4 && y>-0.4) {
    		x = saveX;
    		y = saveY;
    		power = 0;
    		twist = 0;
    		if (!setWheels) startWheelSet();
   // 		setWheels = true;
    	}
    	else {
    		power = (power - 0.3)/0.7;
    		setWheels = false;
    	}
    	
    	if (xBox.getRawButton(5)) power = 0.5*power;

    	saveX = x;
    	saveY = y;
    	
//Field Centric code
    	/*
    	double robotCent = computeAngle(y,x);
    	double bearing = moeGyro.getAngle();
    	if (bearing > 180) bearing = bearing - 360;
    	else if (bearing < -180) bearing = bearing + 360;
    	robotCent = robotCent - bearing;
    	if (robotCent > 180) robotCent = robotCent - 360;
    	else if (robotCent < -180) robotCent = robotCent + 360;
    	getFieldCentricValues(robotCent);
    	x = fieldX;
    	y = fieldY;
    	*/
    	
    
    	
    	xNew = x + twist;
    	yNew = y + twist;
    	angleLF = computeAngle(yNew,xNew);
    	
    	xNew = x + twist;
    	yNew = y - twist;
    	angleRF = computeAngle(yNew,xNew);
    	
    	xNew = x - twist;
    	yNew = y + twist;
    	angleLR = computeAngle(yNew,xNew);
    	
    	xNew = x - twist;
    	yNew = y - twist;
    	angleRR = computeAngle(yNew,xNew);
    	
    	adjustAllWheels(angleLF,angleRF,angleLR,angleRR);
    	
    	if (!nearLF || !nearRF || !nearLR || !nearRR) power = 0;

    	driveAllWheels(power,power,power,power);
    	
    }
    
    void twowaySwerve() {
    	double power;
    	double angleLF;
    	double angleRF;
    	double angleLR;
    	double angleRR;

    	double xNew;
    	double yNew;
    	double x = xBox.getRawAxis(4);
    	double y = -xBox.getRawAxis(5);

 //   	if (Math.abs(y) > Math.abs(x)) power = Math.abs(y);
 //   	else power = Math.abs(x);

    	double twist = xBox.getX();
    	if (twist > 0.5) twist = 0.15;
    	else if (twist < -0.5) twist = -0.15;
    	else twist = 0;

    	power = Math.sqrt(x*x + y*y);
    	power = limit(power);

    	if (power <=0.3) {
  //  	if (x<0.4 && x>-0.4 && y<0.4 && y>-0.4) {
    		x = saveX;
    		y = saveY;
    		power = 0;
    		twist = 0;
    		if (!setWheels) startWheelSet();
  //  		setWheels = true;
    	}
    	else {
    		power = (power - 0.3)/0.7;
    		setWheels = false;
    	}
    	
    	if (xBox.getRawButton(5)) power = 0.5*power;

    

    	double baseAngle = computeBaseAngle(y,x);
    	if (reverse) {
    		x = -x;
    		y = -y;
    		twist = -twist;
    		power = -power;
    	}
    	else {
    		double currentAngle = getCurrentAngle();
    		if (baseAngle < -70 && currentAngle > 70) {
  //  		if (baseAngle - currentAngle > 160) {
  //  			baseAngle = baseAngle + 180;
    			x = -x;
    			y = -y;
    			twist = -twist;
    			power = -power;
    		}
    		else if (baseAngle > 70 && currentAngle < -70) {
 //   		else if (baseAngle - currentAngle < -160) {
 //   			baseAngle = baseAngle - 180;
    			x = -x;
    			y = -y;
    			twist = -twist;
    			power = -power;
    		}

    		//   	if (power < 0) twist = -twist;
    	}
    	
    	saveX = x;
    	saveY = y;
    	
    	xNew = x + twist;
    	yNew = y + twist;
    	angleLF = computeAngle(yNew,xNew);

    	xNew = x + twist;
    	yNew = y - twist;
    	angleRF = computeAngle(yNew,xNew);

    	xNew = x - twist;
    	yNew = y + twist;
    	angleLR = computeAngle(yNew,xNew);

    	xNew = x - twist;
    	yNew = y - twist;
    	angleRR = computeAngle(yNew,xNew);

    	adjustAllWheels(angleLF,angleRF,angleLR,angleRR);

    	if (!nearLF || !nearRF || !nearLR || !nearRR) power = 0;

    	driveAllWheels(power,power,power,power);

    }
    
    void optimizedSwerve()
    {
    	double power;
    	double angleLF;
    	double angleRF;
    	double angleLR;
    	double angleRR;

    	double xNew;
    	double yNew;
    	double x = xBox.getRawAxis(4);
    	double y = -xBox.getRawAxis(5);

 //   	if (Math.abs(y) > Math.abs(x)) power = Math.abs(y);
 //   	else power = Math.abs(x);

    	double twist = xBox.getX();
    	if (twist > 0.5) twist = 0.15;
    	else if (twist < -0.5) twist = -0.15;
    	else twist = 0;

    	power = Math.sqrt(x*x + y*y);
    	power = limit(power);

    	if (power <= 0.3) {
 //   	if (x<0.4 && x>-0.4 && y<0.4 && y>-0.4) {
    		x = saveX;
    		y = saveY;
    		power = 0;
    		twist = 0;
    		if (!setWheels) startWheelSet();
 //   		setWheels = true;
    	}
    	else {
    		power = (power - 0.3)/0.7;
    		setWheels = false;
    	}
    	
    	if (xBox.getRawButton(5)) power = 0.5*power;

    	

    	double startingAngle = getBestAngle(y,x);
    	if (reverse) {
    		x = -x;
    		y = -y;
    		power = -power;
    		twist = -twist;
    	}
    	
    	saveX = x;
    	saveY = y;
    	
    	xNew = x + twist;
    	yNew = y + twist;
    	angleLF = computeAngle(yNew,xNew);

    	xNew = x + twist;
    	yNew = y - twist;
    	angleRF = computeAngle(yNew,xNew);

    	xNew = x - twist;
    	yNew = y + twist;
    	angleLR = computeAngle(yNew,xNew);

    	xNew = x - twist;
    	yNew = y - twist;
    	angleRR = computeAngle(yNew,xNew);

    	adjustAllWheels(angleLF,angleRF,angleLR,angleRR);

    	if (!nearLF || !nearRF || !nearLR || !nearRR) power = 0;

    	driveAllWheels(power,power,power,power);

    }


    void carMode() {
    	double wheelAngle;
    	double x = xBox.getX();
    	double y = -xBox.getRawAxis(5);

    	saveX = 0;
    	saveY = 0.7;

    	if (x >= 0.2 || x <= -0.2) {
    		wheelAngle = (Math.abs(x) - 0.2)*60.;
    		if (x < 0) wheelAngle = -wheelAngle;   		
    	}
    	else wheelAngle = 0;

    	if (y < 0.1 && y > -0.1)  y = 0;
    	if (wheelAngle > 45) wheelAngle = 45;
    	else if (wheelAngle < -45) wheelAngle = -45;

    	adjustAllWheels(wheelAngle,wheelAngle,0,0);

    	if ( deltaLR<4 && deltaRR<4) allSet = true;
    	if (!allSet) {
    		driveAllWheels(0,0,0,0);
    	}
    	else 
    	driveAllWheels(y,y,y,y);

    }
    
    void tankDrive() {
    	
    	double x = xBox.getX();
    	double y = -xBox.getRawAxis(5);
    	
    	if (x<0.1 && x>-0.1) x = 0;
    	if (y<0.1 && y>-0.1) y = 0;
    	
    	saveX = 0;
    	saveY = 0.7;
    	
    	adjustAllWheels(0,0,0,0);
    	
    	if (deltaLF<4 && deltaRF<4 && deltaLR<4 && deltaRR<4) allSet = true;
    	if (!allSet) {
    		driveAllWheels(0,0,0,0);
    	}
    	else {
    	
    	double leftSide = limit(y + x);
    	double rightSide = limit(y - x);
    	
    	driveAllWheels(leftSide,rightSide,leftSide,rightSide);
    	}
    	
    }
    
    void driveStraight(){
    	double twist;
    	if (xBox.getRawButton(2)) {
    		saveX = 0.7;
    		saveY = 0;
    	}
    	else if (xBox.getRawButton(3)) {
    		saveX = 0;
    		saveY = 0.7;
    	}
    	
    	double x = saveX;
    	double y = saveY;
    	double power = -xBox.getRawAxis(5);
    	
    	double bearing = moeGyro.getAngle();
    	double addSum = SmartDashboard.getNumber("DB/Slider 0", 0.0);   //0 for now
    	double proportional = SmartDashboard.getNumber("DB/Slider 1", 0.0);   //1 for now
    	
    	if (bearing > 0.75 && straightSum < 0.1 && straightSum > -0.1) straightSum = -2;
    	else if (bearing < -0.75 && straightSum < 0.1 && straightSum > -0.1) straightSum = 2;
    	
    	if (bearing > 0.8 && bearing < 8) {
    		straightSum = straightSum - addSum;
    	}
    	else if (bearing < -0.8 && bearing > -8) {
    		straightSum = straightSum + addSum;
    	}
    	
    	if (bearing > 0) twist = -proportional*bearing;
    	else if (bearing < 0) twist = -proportional*bearing;
    	else twist = 0;
    	
    	double newTwist = straightSum + twist;
    	
    	if (power < 0.1 && power > -0.1) power = 0;
    	if (power > 0.6) power = 0.6;
    	else if (power < -0.6) power = -0.6;
    	
    	double wheelAngle = computeAngle(y,x);
    	
    	if (wheelAngle > -45 && wheelAngle < 45) {
    		if (power > 0) {
    			adjustAllWheels(wheelAngle+newTwist,wheelAngle+newTwist,wheelAngle,wheelAngle);
    			
    		}
    		else {
    			adjustAllWheels(wheelAngle,wheelAngle,wheelAngle+newTwist,wheelAngle+newTwist);
    		}
    	}
    	else if (wheelAngle > 45) {
    		if (power > 0) {
    			adjustAllWheels(wheelAngle,wheelAngle+newTwist,wheelAngle,wheelAngle+newTwist);
    		}
    		else {
    			adjustAllWheels(wheelAngle+newTwist,wheelAngle,wheelAngle+newTwist,wheelAngle);
    		}
    	}
    	else if (wheelAngle < -45) {
    		if (power > 0) {
    			adjustAllWheels(wheelAngle+newTwist,wheelAngle,wheelAngle+newTwist,wheelAngle);
    		}
    		else {
    			adjustAllWheels(wheelAngle,wheelAngle+newTwist,wheelAngle,wheelAngle+newTwist);
    		}
    	}
    	
    	driveAllWheels(power,power,power,power);
    }

    void turnInPlace() {
    	
    	saveX = 0;
    	saveY = 0.7;
    	
    	adjustAllWheels(45.,-45.,-45.,45.);
    	if (deltaLF<4 && deltaRF<4 && deltaLR<4 && deltaRR<4) allSet = true;
    	if (!allSet) {
    		driveAllWheels(0,0,0,0);
    	}
    	else {
    		double power = xBox.getRawAxis(4);
    		driveAllWheels(power,-power,power,-power);
    	}

    }
    
    void gentleStrafe()
    {
    	double strafePower;
    	
    	adjustAllWheels(90,90,90,90);
    	if (setLF && setRF && setLR && setRR) allSet = true;
    	if (!allSet) {
    		driveAllWheels(0,0,0,0);   		
    	}
    	else {
    		if (xBox.getRawButton(7)) strafePower = -0.35;
    		else strafePower = 0.35;
    		driveAllWheels(strafePower,strafePower,strafePower,strafePower);
    	}
    }
 /*   
    void newAdjustLF(double newAngle) {
    	double oldLF = encoderLF.getRaw()*360.0/256.0;
    	if (oldLF > 180) oldLF = oldLF - 360;
    	else if (oldLF < -180) oldLF = oldLF + 360;
    	deltaLF = (newAngle - oldLF);
    	if (deltaLF > 180) deltaLF = deltaLF - 360;
    	else if (deltaLF < -180) deltaLF = deltaLF + 360;
    	if (Math.abs(deltaLF) < 30) nearLF = true;
    	else nearLF = false;
    	if (Math.abs(deltaLF) <=2.5) setLF = true;
    	else setLF = false;
    	double powerLF = newGetTurnPower(deltaLF);
    	if (setWheels) {
    		if (lastDeltaLF * deltaLF < 0) powerLF = 0.9*powerLF;
    	}
    	twistLeftFront.set(powerLF);
    	lastDeltaLF = deltaLF;
    	
    }*/
    
    void adjustLF(double newAngle) {
    	
    	double oldLF = encoderLF.getRaw()*360.0/256.0;
    	if (oldLF > 180) oldLF = oldLF - 360;
    	else if (oldLF < -180) oldLF = oldLF + 360;
    	dirLF = getDirection(oldLF,newAngle);
    	deltaLF = Math.abs(newAngle - oldLF);
    	if (deltaLF > 180) deltaLF = 360 - deltaLF;
    	if (deltaLF < 30) nearLF = true;
    	else nearLF = false;
    	if (deltaLF <= setPoint) setLF = true;
    	else setLF = false;
    	double powerLF = alternateTurnPower(deltaLF);
    	if (!dirLF) powerLF = -powerLF;
    	/*
    	if (setWheels) {
    		if (sumLF > 4) {
    			sumLF = 0;
				lastDirLF = dirLF;
    		}
    		if (dirLF && !lastDirLF || !dirLF && lastDirLF) overshootLF = true;   		
    		if (overshootLF) powerLF = 0.9*powerLF;  		
    	} */
    	twistLeftFront.set(powerLF);
  //  	lastDirLF = dirLF;
    }

    void adjustRF(double newAngle) {
    	double oldRF = encoderRF.getRaw()*360.0/256.0;
    	if (oldRF > 180) oldRF = oldRF - 360;
    	else if (oldRF < -180) oldRF = oldRF + 360;
    	dirRF = getDirection(oldRF,newAngle);
    	deltaRF = Math.abs(newAngle - oldRF);
    	if (deltaRF > 180) deltaRF = 360 - deltaRF;
    	if (deltaRF < 30) nearRF = true;
    	else nearRF = false;
    	if (deltaRF <= setPoint) setRF = true;
    	else setRF = false;
    	double powerRF = alternateTurnPower(deltaRF);
    	if (!dirRF) powerRF = -powerRF;
    	/*
    	if (setWheels) {
    		if (sumRF > 4) {
    			sumRF = 0;
				lastDirRF = dirRF;
    		}
    		if (dirRF && !lastDirRF || !dirRF && lastDirRF) overshootRF = true;   		
    		if (overshootRF) powerRF = 0.9*powerRF;  		
    	}  */
    	twistRightFront.set(powerRF);
 //   	lastDirRF = dirRF;
    }
    
    void adjustLR(double newAngle) {
    	double oldLR = encoderLR.getRaw()*360.0/256.0;
    	if (oldLR > 180) oldLR = oldLR - 360;
    	else if (oldLR < -180) oldLR = oldLR + 360;
    	dirLR = getDirection(oldLR,newAngle);
    	double deltaLR = Math.abs(newAngle - oldLR);
    	if (deltaLR> 180) deltaLR = 360 - deltaLR;
    	if (deltaLR < 30) nearLR = true;
    	else nearLR = false;
    	if (deltaLR <= setPoint) setLR = true;
    	else setLR = false;
    	double powerLR = alternateTurnPower(deltaLR);
    	if (!dirLR) powerLR = -powerLR;
    	/*
    	if (setWheels) {
    		if (sumLR > 4) {
    			sumLR = 0;
				lastDirLR = dirLR;
    		}
    		if (dirLR && !lastDirLR || !dirLR && lastDirLR) overshootLR = true;   		
    		if (overshootLR) powerLR = 0.9*powerLR;  		
    	}  */
    	twistLeftRear.set(powerLR);
//    	lastDirLR = dirLR;
    }
    
    void adjustRR(double newAngle) {
    	double oldRR = encoderRR.getRaw()*360.0/256.0;
    	if (oldRR > 180) oldRR = oldRR - 360;
    	else if (oldRR < -180) oldRR = oldRR + 360;
    	dirRR = getDirection(oldRR,newAngle);
    	deltaRR = Math.abs(newAngle - oldRR);
    	if (deltaRR > 180) deltaRR = 360 - deltaRR;
    	if (deltaRR < 30) nearRR = true;
    	else nearRR = false;
    	if (deltaRR <= setPoint) setRR = true;
    	else setRR = false;
    	double powerRR = alternateTurnPower(deltaRR);
    	if (!dirRR) powerRR = -powerRR;
    	/*
    	if (setWheels) {
    		if (sumRR > 4) {
    			sumRR = 0;
				lastDirRR = dirRR;
    		}
    		if (dirRR && !lastDirRR || !dirRR && lastDirRR) overshootRR = true;   		
    		if (overshootRR) powerRR = 0.9*powerRR;  		
    	}  */
    	twistRightRear.set(powerRR);
//    	lastDirRR = dirRR;
    }
    
    void adjustAllWheels(double newAngleLF,double newAngleRF,double newAngleLR,double newAngleRR) {
    	adjustLF(newAngleLF);
    	adjustRF(newAngleRF);
    	adjustLR(newAngleLR);
    	adjustRR(newAngleRR);
    }
    
    void driveAllWheels(double powerLF,double powerRF,double powerLR,double powerRR) {
    	driverLeftFront.set(powerLF);
    	driverRightFront.set(powerRF);
    	driverLeftRear.set(powerLR);
    	driverRightRear.set(powerRR);
    }
    
    
    
    boolean getDirection(double oldAngle,double newAngle) {
    	if (newAngle - oldAngle > 180) return false;
    	else if (newAngle - oldAngle < -180) return true;
    	else if (newAngle >= oldAngle) return true;
    	else return false;
    }
    
    double turnPower(double error,boolean dir) {
    	double power;
    	if (error <= 2.5) power = 0;
    	else if (error > 10) {
    		power = 0.02 * error;
    		if (power > 0.45) power = 0.45;
    	}
    	else power = 0.2;
    	if (!dir) power = -power;
    	return power;
    }
    
    double newGetTurnPower(double error) {
    	double power;
    	if (error <= 2.5 && error >= -2.5) power = 0;
    	else if (error > 12 || error < -12) {
    		power = 0.015 * error;
    		if (power > 0.45) power = 0.45;
    		else if (power < -0.45) power = -0.45;
    	}
    	else if (error > 0) power = 0.2;
    	else power = -0.2;
    	return power;
    }

    double alternateTurnPower(double error)  {   //uses absolute value of error
    	double power;
    	
    	double slowAngle = SmartDashboard.getNumber("DB/Slider 2", 0.0);
    	double gain = SmartDashboard.getNumber("DB/Slider 3",0.0);
    	slowAngle = slowAngle*4.;
    	gain = gain * .004;

    	if (error <= setPoint) power = 0;
    	else if (error <= slowAngle) power = 0.2;  	
    	else {
    	 power = 0.2 + gain*(error - slowAngle);   	
    	}
    	if (power > 0.45) power = 0.45;
    	
    	return power;

    
    }
    
    double computeAngle(double y,double x) {
    	double newAngle;
    	newAngle = 180*Math.atan2(y, x)/Math.PI;
    	newAngle = -newAngle + 90;
    	if (newAngle >= 180) newAngle = newAngle - 360;
    	return newAngle;
    }
    
    double computeBaseAngle(double y, double x) {
    	double newAngle;
    	newAngle = 180*Math.atan2(y, x)/Math.PI;
    	newAngle = -newAngle + 90;
    	if (newAngle >= 180) newAngle = newAngle - 360;
    	if (newAngle >= -120 && newAngle <= 120) reverse = false;
    	else if (newAngle > 120) {
    		newAngle = newAngle - 180;
    		reverse = true;
    	}
    	else {
    		newAngle = newAngle + 180;
    		reverse = true;
    	}
    	return newAngle;
    }
    
    double getBestAngle(double y,double x) {
     	double newAngle;
     	double currentAngle = getCurrentAngle();
    	newAngle = 180*Math.atan2(y, x)/Math.PI;
    	newAngle = -newAngle + 90;
    	if (newAngle >= 180) newAngle = newAngle - 360;
    	if (newAngle - currentAngle > 110) {
    		newAngle = newAngle - 180;
    		reverse = true;
    	}
    	else if (newAngle - currentAngle < -110) {
    		newAngle = newAngle + 180;
    		reverse = true;
    	}
    	else reverse = false;
    	return newAngle;
    	
    }
    
    double getCurrentAngle()  {
    	double oldLF = encoderLF.getRaw()*360.0/256.0;
    	if (oldLF > 180) oldLF = oldLF - 360;
    	else if (oldLF < -180) oldLF = oldLF + 360;
    	return oldLF;
    	
    }
    
    void getFieldCentricValues(double fieldAngle) {
    	double newAngle;
    	if (fieldAngle >= 0 && fieldAngle < 90) {
    		newAngle = (90 - fieldAngle)*Math.PI/180;
    		fieldY = Math.sin(newAngle);
    		fieldX = Math.cos(newAngle);
    	}
    	else if (fieldAngle > 90) {
    		newAngle = (fieldAngle - 90)*Math.PI/180.;
    		fieldY = -Math.sin(newAngle);
    		fieldX = Math.cos(newAngle);
    	}
    	else if (fieldAngle < 0 && fieldAngle >= -90) {
    		newAngle = (fieldAngle + 90)*Math.PI/180.;
    		fieldY = Math.sin(newAngle);
    		fieldX = -Math.cos(newAngle);
    	}
    	else if (fieldAngle < -90) {
    		newAngle = (-fieldAngle - 90)*Math.PI/180;
    		fieldY = -Math.sin(newAngle);
    		fieldX = -Math.cos(newAngle);
    	}
    }
    
    void startWheelSet() {
    	allSet = false;
    	setWheels = true;
    	overshootLF = false;
    	overshootRF = false;
    	overshootLR = false;
    	overshootRR = false;
    	sumLF = 5;
    	sumRF = 5;
    	sumLR = 5;
    	sumRR = 5;
    }
    
    void encoderRevolutionCheck() {
    	int pulsesLF = encoderLF.getRaw();
    	int pulsesRF = encoderRF.getRaw();
    	int pulsesLR = encoderLR.getRaw();
    	int pulsesRR = encoderRR.getRaw();
    	if (pulsesLF > 256 || pulsesLF < -256) encoderLF.reset();
    	if (pulsesRF > 256 || pulsesRF < -256) encoderRF.reset();
    	if (pulsesLR > 256 || pulsesLR < -256) encoderLR.reset();
    	if (pulsesRR > 256 || pulsesRR < -256) encoderRR.reset();
    	
    }
    
    double limit(double value) {
    	if (value > 1) return 1;
    	else if (value < -1) return -1;
    	else return value;
    }
    
    void autoStraight(double wheelAngle,double power) {
    	
    	double twist;
    	
    	double bearing = moeGyro.getAngle();
    	double addSum = SmartDashboard.getNumber("DB/Slider 0", 0.0);   //0 for now
    	double proportional = SmartDashboard.getNumber("DB/Slider 1", 0.0);   //1 for now
    	
    	if (bearing > 0.75 && straightSum < 0.1 && straightSum > -0.1) straightSum = -2;
    	else if (bearing < -0.75 && straightSum < 0.1 && straightSum > -0.1) straightSum = 2;
    	
    	if (bearing > 0.8 && bearing < 8) {
    		straightSum = straightSum - addSum;
    	}
    	else if (bearing < -0.8 && bearing > -8) {
    		straightSum = straightSum + addSum;
    	}
    	
    	if (bearing > 0) twist = -proportional*bearing;
    	else if (bearing < 0) twist = -proportional*bearing;
    	else twist = 0;
    	
    	double newTwist = straightSum + twist;
    	
     	
    	if (wheelAngle > -45 && wheelAngle < 45) {
    		if (power > 0) {
    			adjustAllWheels(wheelAngle+newTwist,wheelAngle+newTwist,wheelAngle,wheelAngle);
    			
    		}
    		else {
    			adjustAllWheels(wheelAngle,wheelAngle,wheelAngle+newTwist,wheelAngle+newTwist);
    		}
    	}
    	else if (wheelAngle > 45) {
    		if (power > 0) {
    			adjustAllWheels(wheelAngle,wheelAngle+newTwist,wheelAngle,wheelAngle+newTwist);
    		}
    		else {
    			adjustAllWheels(wheelAngle+newTwist,wheelAngle,wheelAngle+newTwist,wheelAngle);
    		}
    	}
    	else if (wheelAngle < -45) {
    		if (power > 0) {
    			adjustAllWheels(wheelAngle+newTwist,wheelAngle,wheelAngle+newTwist,wheelAngle);
    		}
    		else {
    			adjustAllWheels(wheelAngle,wheelAngle+newTwist,wheelAngle,wheelAngle+newTwist);
    		}
    	}
    	
    }
    
    void testRoutine() {
    	double x = xBox.getX();
    	double y = -xBox.getRawAxis(5);
    	
    	if (x<0.1 && x>-0.1) x = 0;
    	if (y<0.1 && y>-0.1) y = 0;
    	
    	twistLeftFront.set(x);
    	twistRightFront.set(x);;
    	twistLeftRear.set(x);
    	twistRightRear.set(x);
    	
    	driverLeftFront.set(y);
    	driverRightFront.set(y);
    	driverLeftRear.set(y);
    	driverRightRear.set(y);
    }
   
    void wheelAngleTestRoutine() {
   //	int countTime = 0;
    	double power;
    	double x = xBox.getRawAxis(4);
    	double y = -xBox.getRawAxis(5);

    	power = Math.sqrt(x*x + y*y);
    	power = limit(power);

    	if (power <= 0.5) {
 //   	if (x<0.4 && x>-0.4 && y<0.4 && y>-0.4) {
    		x = saveX;
    		y = saveY;	
  //  		power = 0;
    		if (!setWheels) {
    			startWheelSet();
    			countTime = 0;
    		}
   // 		setWheels = true;
    	}
    	else {
 //   		power = (power - 0.3)/0.7;
    		setWheels = false;
 
    	}
    	
 //   	if (xBox.getRawButton(5)) power = 0.5*power;

    	saveX = x;
    	saveY = y;

    	if (setWheels) {
    		double wheelAngle = computeAngle(y,x);
    		adjustAllWheels(wheelAngle,wheelAngle,wheelAngle,wheelAngle);

    		if (setLF && setRF && setLR && setRR) allSet = true;
    		if (allSet) {
    			countTime++;
    			if (countTime < 100)  driveAllWheels(0.4,0.4,0.4,0.4);
    			else driveAllWheels(0,0,0,0);
    		}
    		else driveAllWheels(0,0,0,0);

    	}
    	else driveAllWheels(0,0,0,0);

    }
}
