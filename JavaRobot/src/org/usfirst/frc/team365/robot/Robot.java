package org.usfirst.frc.team365.robot;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	DriverStation ds;
	CameraServer server;
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
	
	Encoder encoderLF = new Encoder(4,5,true,EncodingType.k1X);
	Encoder encoderRF = new Encoder(0,1,true,EncodingType.k1X);
	Encoder encoderLR = new Encoder(6,7,true,EncodingType.k1X);
	Encoder encoderRR = new Encoder(8,9,true,EncodingType.k1X);
	
	Encoder distEncoder = new Encoder(2,3,true,EncodingType.k1X);
	Encoder dist2Encoder = new Encoder(17,15,true,EncodingType.k1X);
	
	Encoder liftEncoder = new Encoder(14,16,false,EncodingType.k1X);
	
	DigitalInput leftEye = new DigitalInput(11);
	DigitalInput rightEye = new DigitalInput(10);
	DigitalInput limitBottom = new DigitalInput(13);
	DigitalInput limitTop = new DigitalInput(12);
	
	AnalogInput distIR = new AnalogInput(1);
	
	DoubleSolenoid canLeft = new DoubleSolenoid(0,1);
	DoubleSolenoid canRight = new DoubleSolenoid(2,3);
	Solenoid finger = new Solenoid(4);
	
	Solenoid armLeft = new Solenoid(5);
	Solenoid armRight = new Solenoid(6);
	
	Gyro moeGyro = new Gyro(0);
	
	Joystick xBox = new Joystick(0);
	Joystick funBox = new Joystick(1);
	Joystick canBox = new Joystick(2);
	
	int autoLoop;
	int autoStep;
	int disabledLoop;
	int teleopLoop;
	int stackStep;
	int loopCount;
//	boolean lastDirLF;
//	boolean lastDirRF;
//	boolean lastDirLR;
//	boolean lastDirRR;
	double deltaLF;
	double deltaRF;
	double deltaLR;
	double deltaRR;
//	double sumLF;
//	double sumRF;
//	double sumLR;
//	double sumRR;
	int countTime;
//	boolean overshootLF;
//	boolean overshootRF;
//	boolean overshootLR;
//	boolean overshootRR;
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
	double lowBearing;
	double highBearing;
	double avgBearing;
	double sumBearing;
	boolean last1;
	boolean last2;
	boolean last3;
	boolean last4;
	boolean last5;
	boolean last7;
	boolean last8;
	double lastAxisZ;
	double lastFunZ;
	boolean lastCan9;
	int lastPOV;
	int lastFunPOV;
	int lastCanPOV;
	boolean dirLF;
	boolean dirRF;
	boolean dirLR;
	boolean dirRR;
	boolean reverse;
	boolean allSet;
	boolean setWheels;
	boolean swerveMode;
	boolean turnAround;
	double fieldX;
	double fieldY;
	double printAngle;
	double targetDist;
	double holdP;
	
	final int toteOne = 15;
	final int toteTwo = 1000;   //1016;
	final int toteThree = 2000;  //2070;
	final int toteFour = 3000;  //3128;
	
	final int ENCODERcount = 256;
	
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
	
	public Robot() {
		ds = DriverStation.getInstance();
		server = CameraServer.getInstance();
		server.setSize(2);
		server.setQuality(50);
		server.startAutomaticCapture("cam0");
	
	}
    public void robotInit() {
    	
    	turnLF.enableBrakeMode(true);
    	turnRF.enableBrakeMode(true);
    	turnLR.enableBrakeMode(true);
    	turnRR.enableBrakeMode(true);
    	
    	driveLF.enableBrakeMode(false);
    	driveRF.enableBrakeMode(false);
    	driveLR.enableBrakeMode(false);
    	driveRR.enableBrakeMode(false);
    	
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
    	}
    	
    	if (xBox.getRawButton(10)) {
    		distEncoder.reset();
    		dist2Encoder.reset();
    	}
    	
    	if (funBox.getRawButton(8)) {
    		liftEncoder.reset();
    	}
    	
    	if (xBox.getRawButton(7)) {
    		moeGyro.initGyro();
  //  		moeGyro.reset();
    	}
    	
     	setPoint = SmartDashboard.getNumber("DB/Slider 3", 0.0);
    	if (setPoint < 1) setPoint = 2;
    	
    	if (disabledLoop % 20 == 0) {
    		
    		SmartDashboard.putNumber("encoderLF", encoderLF.getRaw());
    		SmartDashboard.putNumber("encoderRF", encoderRF.getRaw());
    		SmartDashboard.putNumber("encoderLR", encoderLR.getRaw());
    		SmartDashboard.putNumber("encoderRR", encoderRR.getRaw());
    		
    		SmartDashboard.putNumber("distance", distEncoder.getRaw());
    		SmartDashboard.putNumber("dist2", dist2Encoder.getRaw());
    		SmartDashboard.putNumber("IRvalue", distIR.getAverageVoltage());
    		
    		SmartDashboard.putBoolean("leftEye", leftEye.get());
    		SmartDashboard.putBoolean("rightEye", rightEye.get());
    		
    		SmartDashboard.putNumber("lift", liftEncoder.getRaw());
    		
    		SmartDashboard.putNumber("gyro", moeGyro.getAngle());
    		
    		SmartDashboard.putNumber("POV", xBox.getPOV());
    		
    		SmartDashboard.putNumber("twist", xBox.getX());
    		
    		SmartDashboard.putBoolean("bottom", limitBottom.get());
        	SmartDashboard.putBoolean("top", limitTop.get());
    	}
    	
    

    }
    
    /**
     * This function is run once each time the robot enters autonomous mode
     */
    public void autonomousInit() {
		autoStep = 1;
    	autoLoop = 0;
    	loopCount = 0;
    	moeGyro.reset();
    	distEncoder.reset();
    	straightSum = 0;
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
  //  	if(autoLoop < 100) //Check if we've completed 100 loops (approximately 2 seconds)
//		{
	
			autoLoop++;
			loopCount++;
			autoStackRoutine();
//		}
		
    }
    
    /**
     * This function is called once each time the robot enters tele-operated mode
     */
    public void teleopInit(){
    	teleopLoop = 0;
    	lastPOV = -1;
    	lastAxisZ = 0;
    	lastFunZ = 0;
    	lastCan9 = false;
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
    	turnAround = false;
    	setWheels = false;
    	allSet = false;
    	armLeft.set(false);
    	armRight.set(false);
    	maxCurLF = 0;
    	maxCurRF = 0;
    	maxCurLR = 0;
    	maxCurRR = 0;
 //   	saveX = 0.;
//    	saveY = 0.7;
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
    	
 //   	setPoint = SmartDashboard.getNumber("DB/Slider 3", 0.0);
 //   	if (setPoint < 1) setPoint = 2;
 //   	setPoint = 2;
    	
 //   	System.out.print(setPoint);
    	
    	boolean new1 = xBox.getRawButton(1);
    	boolean new2 = xBox.getRawButton(2);
    	boolean new3 = xBox.getRawButton(3);
    	boolean new4 = xBox.getRawButton(4);
    	boolean new5 = xBox.getRawButton(5);
    	boolean new7 = xBox.getRawButton(7);
    	boolean new8 = xBox.getRawButton(8);
    	
    	double newFunZ = funBox.getZ();
    	double newAxisZ = xBox.getZ();
    	int newPOV = xBox.getPOV();
    	int newFunPOV = funBox.getPOV();
    	int newCanPOV = canBox.getPOV();
    	boolean newCan9 = canBox.getRawButton(9);
    	
    	teleopLoop++;
    	double robotBearing = moeGyro.getAngle();
    	if (robotBearing >=360 || robotBearing <=-360) moeGyro.reset();
    	
    	encoderRevolutionCheck();
    	controlLifter();
    	canTainerControls();
  //  	crabDrive360();
 //   	testRoutine();
 //   	wheelAngleTestRoutine();
    	
   	
    	if (newAxisZ > 0.5) {
    		if (lastAxisZ <= 0.5) {
    			startWheelSet();
    			moeGyro.reset();
    			straightSum = 0;
    			turnAround = false;  			
    		}
    		driveStraight();
    	}
    	else if (new2) {
    		turnAround = false;
    		if (!last2) {
    			stackStep = 1;
    		}
    		moveStackToPlatform();
    	}
    	else if (newFunZ > 0.5) {
    		turnAround = false;
     		adjustAllWheels(0,0,0,0);
    		if (lastFunZ <= 0.5) {
    			loopCount = 0;
    			stackStep = 1;
    			distEncoder.reset();
    			driveAllWheels(0,0,0,0);
    		}
    		lineUpAtFeeder();
    	}
   // 	else if (newCanPOV == 0 || newCanPOV == 180) {
  //  		turnAround = false;
  //  		adjustAllWheels(0,0,0,0);
  //  		if (lastCanPOV == -1) {
  //  			loopCount = 0;
    //			stackStep = 1;
    	//		distEncoder.reset();
    //			driveAllWheels(0,0,0,0);
    //		}
    	//	if (newCanPOV == 0) lineUpAtFeeder();
    //		else if (newCanPOV == 180) stackAtFeeder();
    //	}
    	else if (new3) { 
    		turnAround = false;
    		if (!last3) {
    			stackStep = 1;
    		}
    		carModeBackUp();
    	}
    	else if (newPOV != -1) {
    		turnAround = false;
    		if (lastPOV == -1) {
    			startWheelSet();
    		}
    		slowPOVDrive(newPOV);
    	}
    	else if (new7) {
    		turnAround = false;
    		if (!last7) {
    			startWheelSet();
    		}
    		slightTurnLeft();
    	}
    	else if (new8) {
    		turnAround = false;
    		if (!last8) {
    			startWheelSet();
    		}
    		slightTurnRight();
    	}
    	else if (!swerveMode) {
    		 	  carMode();
    	//	tankDrive();
    	}
    	else {
 //   		crabDrive360();
 //   		swerveDrive360();
 //   		optimizedSwerve();
    		newOptimizedSwerve();
    	}

    	if (new1 && !last1) {
    		swerveMode = false;
    	//	allSet = false;
    		startWheelSet();
    		turnAround = false;
    	}
    	else if (new4 && !last4) {
    		turnAround = false;
    		swerveMode = true;
    	}
    

    	lastPOV = newPOV;
    	lastFunPOV = newFunPOV;
    	lastCanPOV = newCanPOV;
    	lastCan9 = newCan9;
    	last1 = new1;
    	last2 = new2;
    	last3 = new3;
    	last4 = new4;
    	last5 = new5;
    	last7 = new7;
    	last8 = new8;
    	lastAxisZ = newAxisZ;
    	lastFunZ = newFunZ;

    	if (teleopLoop % 20 == 0) {

    		boolean newSet;

    		SmartDashboard.putNumber("encoderLF", encoderLF.getRaw());
    		SmartDashboard.putNumber("encoderRF", encoderRF.getRaw());
    		SmartDashboard.putNumber("encoderLR", encoderLR.getRaw());
    		SmartDashboard.putNumber("encoderRR", encoderRR.getRaw());
    		
    		SmartDashboard.putNumber("distance", distEncoder.getRaw());
    		SmartDashboard.putNumber("dist2", dist2Encoder.getRaw());
    		SmartDashboard.putNumber("IRvalue", distIR.getAverageVoltage());
    		
    		SmartDashboard.putBoolean("leftEye", leftEye.get());
    		SmartDashboard.putBoolean("rightEye", rightEye.get());
    		
    		SmartDashboard.putNumber("lift", liftEncoder.getRaw());
    		
    		SmartDashboard.putNumber("gyro", moeGyro.getAngle());
    		
    		SmartDashboard.putNumber("wheelAngle", printAngle);
    		SmartDashboard.putNumber("POV", xBox.getPOV());
    		
    		SmartDashboard.putNumber("curLF", maxCurLF);
        	SmartDashboard.putNumber("curRF", maxCurRF);
        	SmartDashboard.putNumber("curLR", maxCurLR);
        	SmartDashboard.putNumber("curRR", maxCurRR);
    		
        	SmartDashboard.putNumber("twist", xBox.getX());
        	
        	SmartDashboard.putBoolean("bottom", limitBottom.get());
        	SmartDashboard.putBoolean("top", limitTop.get());
        	
        	SmartDashboard.putNumber("xtra", distEncoder.getRaw() % 400);
    	   		
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
    	
    	double curLF = driveLF.getOutputCurrent();
    	if (curLF > maxCurLF)  maxCurLF = curLF;
    	double curRF = driveRF.getOutputCurrent();
    	if (curRF > maxCurRF) maxCurRF = curRF;
    	double curLR = driveLR.getOutputCurrent();
    	if (curLR > maxCurLR) maxCurLR = curLR;
    	double curRR = driveRR.getOutputCurrent();
    	if (curRR > maxCurRR) maxCurRR = curRR;
    	
    
    	
    	printAngle = wheelAngle;

    }
    
	void swerveDrive360() {
    	double power;
    	double reduce;
    	
    	double twist;
    	double angleLF;
    	double angleRF;
    	double angleLR;
    	double angleRR;
    	double powerLF;
    	double powerRF;
    	double powerLR;
    	double powerRR;
    	double ratioP;
    	
    	double xNew;
    	double yNew;
    	double x = xBox.getRawAxis(4);
    	double y = -xBox.getRawAxis(5);
    	
    	power = Math.sqrt(x*x + y*y);
    	power = limit(power);
    	
    	
    	double newTwist = xBox.getX();
    

    	if (power <=0.3) {
    		if (newTwist > 0.5 || newTwist < -0.5 || turnAround) {
    			x = 0;
    			y = 0;
    			twist = 0.5;
    			turnAround = true;
    			power = 0.7 * newTwist;
    			if (xBox.getRawButton(5)) power = 0.5 * power;
    		}
    		else {
    			turnAround = false;
    			x = saveX;
        		y = saveY;
        		power = 0;
    			twist = 0;
    		}
    		if (!setWheels) startWheelSet();
   // 		setWheels = true;
    	}
    	else {
 //   		power = (power - 0.3)/0.7;
    		setWheels = false;
    		turnAround = false;
    		if (newTwist > 0.2 || newTwist < -0.2) {   //limit twist 
        		twist = newTwist * 0.35;
        	}
        	else twist = 0;       	
    //    	if (twist < -power) twist = -power + 0.05;
    //    	if (twist > power) twist = power - 0.05;
    	}
    	
 //   	if (xBox.getRawButton(5)) power = 0.5*power;

    	saveX = x;
    	saveY = y;
    	
//Field Centric code
 /*   	
    	if (!turnAround) {
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
    	}*/
    	
   // just in case x = twist I don't want to get a condition where xNew = 0 and yNew = 0; 	
    	if (Math.abs(twist) > 0.1) {
    		if (x - twist < .0005 && x - twist > -.0005) {
    			if (x >= twist) twist = twist - .001;
    			else twist = twist + .001;
    		}
    	}
    
    	
    	xNew = x + twist;
    	yNew = y + twist;
    	angleLF = computeAngle(yNew,xNew);
    	powerLF = computeDrivePower(yNew,xNew);
  //  	powerLF = limit(powerLF);
    	
    	xNew = x + twist;
    	yNew = y - twist;
    	angleRF = computeAngle(yNew,xNew);
    	powerRF = computeDrivePower(yNew,xNew);
 //   	powerRF = limit(powerRF);
    	
    	xNew = x - twist;
    	yNew = y + twist;
    	angleLR = computeAngle(yNew,xNew);
    	powerLR = computeDrivePower(yNew,xNew);
 //   	powerLR = limit(powerLR);

    	xNew = x - twist;
    	yNew = y - twist;
    	angleRR = computeAngle(yNew,xNew);
    	powerRR = computeDrivePower(yNew,xNew);
 //   	powerRR = limit(powerRR);
    	
    	adjustAllWheels(angleLF,angleRF,angleLR,angleRR);
    	
   
    	
    	if (!nearLF || !nearRF || !nearLR || !nearRR) {
    		driveAllWheels(0,0,0,0);
    	}
 //   	else driveAllWheels(powerLF,powerRF,powerLR,powerRR);
    	
    	else {
    		if (turnAround) {
    			if (Math.abs(power) < .05) driveAllWheels(0,0,0,0);
    			else {
    				if (power*driveLF.get() <= 0) ratioP = 0;
    				else ratioP = (driveLF.get()/power);
    				if (ratioP < 1) ratioP = ratioP + 0.1;
    				if (ratioP > 1) ratioP = 1;
    				driveAllWheels(ratioP*power,ratioP*power,ratioP*power,ratioP*power);
    			}
    		}
    		else if (power > 0.3) {
    			double maxPower = getMaxPower(powerLF,powerRF,powerLR,powerRR);
    			if (maxPower > 0.75) reduce = 0.75;
    			else reduce = 1.0;
    			if (maxPower < 1) maxPower = 1;
    			if (xBox.getRawButton(5)) maxPower = 0.5/maxPower;
    			else maxPower = reduce/maxPower;
    			powerLF = powerLF * maxPower;
    			powerRF = powerRF * maxPower;
    			powerLR = powerLR * maxPower;
    			powerRR = powerRR * maxPower;
    			if (Math.abs(powerLF) > 0.2) {
    				ratioP = Math.abs(driveLF.get()/powerLF);
    				if (ratioP < 1) ratioP = ratioP + 0.1;
    				ratioP = limit(ratioP);
    			}
    			else {
    				ratioP = Math.abs(driveRF.get()/powerRF);
    				if (ratioP < 1) ratioP = ratioP + 0.1;
    				ratioP = limit(ratioP);
    			}
    			driveAllWheels(ratioP*powerLF,ratioP*powerRF,ratioP*powerLR,ratioP*powerRR);
    		}
    		else driveAllWheels(0,0,0,0);
    		
    	}

 //   	driveAllWheels(power,power,power,power);
    	
    	printAngle = angleLF;
    	
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
    	
    	printAngle = angleLF;

    }
    
    void optimizedSwerve()
    {
    	double power;
    	double twist;
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

     	double newTwist = xBox.getX();
    	if (newTwist > 0.2 || newTwist < -0.2) {
    		twist = newTwist * 0.35;
    	}
 //   	if (twist > 0.7) twist = 0.2;
 //   	else if (twist > 0.4) twist = 0.15;
//    	else if (twist < -0.7) twist = -0.2;
 //   	else if (twist < -0.5) twist = -0.15;
    	else twist = 0;


    	power = Math.sqrt(x*x + y*y);
    	power = limit(power);

    	if (power <=0.3) {
    		//  	if (x<0.4 && x>-0.4 && y<0.4 && y>-0.4) {

    		if (newTwist > 0.5 || newTwist < -0.5 || turnAround) {
    			x = 0;
    			y = 0;
    			twist = 0.5;
    			// 			twist = Math.abs(newTwist);
    			//			if (twist <=0.5) twist = 0.5;
    			turnAround = true;
    			power = 0.6 * newTwist;
    		}
    		else {
    			turnAround = false;
    			x = saveX;
    			y = saveY;
    			power = 0;
    			twist = 0;
    		}
    		if (!setWheels) startWheelSet();
    		// 		setWheels = true;
    	}
    	else {
    		power = (power - 0.3)/0.7;
    		setWheels = false;
    		turnAround = false;
    	}

    	if (xBox.getRawButton(5)) power = 0.5*power;
    	else if (xBox.getRawButton(6))  power = 0.7*power;



    	boolean reverseAngleDirection = chooseBestAngle(y,x);
    	if (reverseAngleDirection) {
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

    	driveAllWheels(power,0.9*power,power,power);
    	
    	double curLF = driveLF.getOutputCurrent();
    	if (curLF > maxCurLF)  maxCurLF = curLF;
    	double curRF = driveRF.getOutputCurrent();
    	if (curRF > maxCurRF) maxCurRF = curRF;
    	double curLR = driveLR.getOutputCurrent();
    	if (curLR > maxCurLR) maxCurLR = curLR;
    	double curRR = driveRR.getOutputCurrent();
    	if (curRR > maxCurRR) maxCurRR = curRR;
    	
    	printAngle = angleLF;

    }

    void newOptimizedSwerve() {
    	double power;
    	double reduce;

    	double twist;
    	double angleLF;
    	double angleRF;
    	double angleLR;
    	double angleRR;
    	double powerLF;
    	double powerRF;
    	double powerLR;
    	double powerRR;
    	double ratioP;

    	double xNew;
    	double yNew;
    	double x = xBox.getRawAxis(4);
    	double y = -xBox.getRawAxis(5);

    	power = Math.sqrt(x*x + y*y);
    	power = limit(power);


    	double newTwist = xBox.getX();


    	if (power <=0.3) {
    		if (newTwist > 0.5 || newTwist < -0.5 || turnAround) {
    			x = 0;
    			y = 0;
    			twist = 0.5;
    			turnAround = true;
    			power = 0.7 * newTwist;
    			if (xBox.getRawButton(5)) power = 0.5 * power;
    		}
    		else {
    			turnAround = false;
    			x = saveX;
    			y = saveY;
    			power = 0;
    			twist = 0;
    		}
    		if (!setWheels) startWheelSet();
    		// 		setWheels = true;
    	}
    	else {
    		//   		power = (power - 0.3)/0.7;
    		setWheels = false;
    		turnAround = false;
    		if (newTwist > 0.2|| newTwist < -0.2) {   //limit twist 
    			twist = newTwist * 0.35;
    		}
    		else twist = 0;       	
    	//	if (twist < -power) twist = -power + 0.05;
    	//	if (twist > power) twist = power - 0.05;
    	}

    	//   	if (xBox.getRawButton(5)) power = 0.5*power;

    	boolean reverseAngleDirection = chooseBestAngle(y,x);
    	if (reverseAngleDirection) {
    		x = -x;
    		y = -y;
    		power = -power;
    		twist = -twist;
    	}


    	saveX = x;
    	saveY = y;

    	//Field Centric code
    	/*   	
    	if (!turnAround) {
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
    	}*/

    	// just in case x = twist I don't want to get a condition where xNew = 0 and yNew = 0; 	
    	if (Math.abs(twist) > 0.1) {
    		if (x - twist < .0005 && x - twist > -.0005) {
    			if (x >= twist) twist = twist - .001;
    			else twist = twist + .001;
    		}
    	}


    	xNew = x + twist;
    	yNew = y + twist;
    	angleLF = computeAngle(yNew,xNew);
    	powerLF = computeDrivePower(yNew,xNew);
    	if (reverseAngleDirection) powerLF = -powerLF;

    	xNew = x + twist;
    	yNew = y - twist;
    	angleRF = computeAngle(yNew,xNew);
    	powerRF = computeDrivePower(yNew,xNew);
    	if (reverseAngleDirection) powerRF = -powerRF;

    	xNew = x - twist;
    	yNew = y + twist;
    	angleLR = computeAngle(yNew,xNew);
    	powerLR = computeDrivePower(yNew,xNew);
    	if (reverseAngleDirection) powerLR = -powerLR;

    	xNew = x - twist;
    	yNew = y - twist;
    	angleRR = computeAngle(yNew,xNew);
    	powerRR = computeDrivePower(yNew,xNew);
    	if (reverseAngleDirection) powerRR = -powerRR;

    	adjustAllWheels(angleLF,angleRF,angleLR,angleRR);



    	if (!nearLF || !nearRF || !nearLR || !nearRR) {
    		driveAllWheels(0,0,0,0);
    	}
    	//   	else driveAllWheels(powerLF,powerRF,powerLR,powerRR);

    	else {
    		if (turnAround) {
    			if (Math.abs(power) < .05) driveAllWheels(0,0,0,0);
    			else {
    				if (power*driveLF.get() <= 0) ratioP = 0;
    				else ratioP = (driveLF.get()/power);
    				if (ratioP < 1) ratioP = ratioP + 0.1;
    				if (ratioP > 1) ratioP = 1;
    				driveAllWheels(ratioP*power,ratioP*power,ratioP*power,ratioP*power);
    			}
    		}
    		else if (Math.abs(power) > 0.3) {
    			double maxPower = getMaxPower(powerLF,powerRF,powerLR,powerRR);
    			if (maxPower > 0.75) reduce = 0.75;
    			else reduce = 1.0;
    			if (maxPower < 1) maxPower = 1;
    			if (xBox.getRawButton(5)) maxPower = 0.5/maxPower;
    			else maxPower = reduce/maxPower;
    			powerLF = powerLF * maxPower;
    			powerRF = powerRF * maxPower;
    			powerLR = powerLR * maxPower;
    			powerRR = powerRR * maxPower;
    			
    			if (Math.abs(powerLF) > 0.2) {
    				if (powerLF*driveLF.get() <= 0) ratioP = 0;
    				else ratioP = (driveLF.get()/powerLF);
    				if (ratioP < 1) ratioP = ratioP + 0.1;
    				if (ratioP > 1) ratioP = 1;
    			}
    			else {
    				if (powerRF*driveRF.get() <= 0) ratioP = 0;
    				else ratioP = (driveRF.get()/powerRF);
    				if (ratioP < 1) ratioP = ratioP + 0.1;
    				if (ratioP > 1) ratioP = 1;
    			}
    			driveAllWheels(ratioP*powerLF,0.9*ratioP*powerRF,ratioP*powerLR,ratioP*powerRR);
  //  			driveAllWheels(powerLF,powerRF,powerLR,powerRR);
    		}
    		else driveAllWheels(0,0,0,0);

    	}
    	
    	double curLF = driveLF.getOutputCurrent();
    	if (curLF > maxCurLF)  maxCurLF = curLF;
    	double curRF = driveRF.getOutputCurrent();
    	if (curRF > maxCurRF) maxCurRF = curRF;
    	double curLR = driveLR.getOutputCurrent();
    	if (curLR > maxCurLR) maxCurLR = curLR;
    	double curRR = driveRR.getOutputCurrent();
    	if (curRR > maxCurRR) maxCurRR = curRR;

    	//   	driveAllWheels(power,power,power,power);

    	printAngle = angleLF;

    }


    void carMode() {
    	double wheelAngle;
    	double x = xBox.getX();
    	double y = -xBox.getRawAxis(5);

    	saveX = 0;
    	saveY = 0.7;

    	if (x >= 0.2 || x <= -0.2) {
    		wheelAngle = (Math.abs(x) - 0.2)*75.;
    		if (x < 0) wheelAngle = -wheelAngle;   		
    	}
    	else wheelAngle = 0;

    	if (y < 0.1 && y > -0.1)  y = 0;
    	if (wheelAngle > 60) wheelAngle = 60;
    	else if (wheelAngle < -60) wheelAngle = -60;

    	adjustAllWheels(wheelAngle,wheelAngle,0,0);

    	if ( deltaLR<4 && deltaRR<4) allSet = true;
    	if (!allSet) {
    		driveAllWheels(0,0,0,0);
    	}
    	else 
    	driveAllWheels(y,y,y,y);
    	
    	printAngle = wheelAngle;

    }
    
   
	void tankDrive() {
    	
    	double x = xBox.getRawAxis(4);
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
    	
    	printAngle = 0;
    	
    }
    
    void driveStraight(){
    	double twist;
    	/*
    	if (xBox.getRawButton(2)) {
    		saveX = 0.7;
    		saveY = 0;
    	}
    	else if (xBox.getRawButton(3)) {
    		saveX = 0;
    		saveY = 0.7;
    	}
    	*/
    	double x = saveX;
    	double y = saveY;
    	double power = -xBox.getRawAxis(5);
    	
    	double bearing = moeGyro.getAngle();
    	
    	double proportional = 2. * SmartDashboard.getNumber("DB/Slider 0", 0.0);   //1 for now
    	double startSum = SmartDashboard.getNumber("DB/Slider 1", 0.0);      //try 1 to start
    	double maxTwist = 2 *SmartDashboard.getNumber("DB/Slider 2", 0.0);   //3 for now
    	
    	if (bearing > 0.75 && straightSum < 0.1 && straightSum > -0.1) straightSum = -startSum;
    	else if (bearing < -0.75 && straightSum < 0.1 && straightSum > -0.1) straightSum = startSum;
    	
 //   	if (bearing > 0.8 && bearing < 8) {
 //   		straightSum = straightSum - addSum;
  //  	}
  //  	else if (bearing < -0.8 && bearing > -8) {
 //   		straightSum = straightSum + addSum;
 //   	}
    	
 //   	if (bearing > 0) twist = -proportional*bearing;
 //   	else if (bearing < 0) twist = -proportional*bearing;
 //   	else twist = 0;
    	
    	twist = -proportional*bearing;
    	
 //   	twist = -8 * bearing;
    	
  //  	if (twist > maxTwist) twist = maxTwist;
  //  	else if (twist < -maxTwist) twist = -maxTwist;
    	
    	double newTwist = straightSum + twist;
  //  	if (newTwist > 3) newTwist = 3;
 //   	else if (newTwist < -3) newTwist = -3;
    	
    	if (power < 0.1 && power > -0.1) power = 0;
    	if (power > 0.5) power = 0.5;
    	else if (power < -0.5) power = -0.5;
    	
    	double wheelAngle = computeAngle(y,x);
    	
    	if (wheelAngle > -45 && wheelAngle < 45) {
    		if (power > 0) {
//    			adjustAllWheels(wheelAngle+newTwist,wheelAngle+newTwist,wheelAngle,wheelAngle);
    			turnWheels(wheelAngle+newTwist,wheelAngle+newTwist,wheelAngle,wheelAngle);
    		}
    		else {
 //   			adjustAllWheels(wheelAngle,wheelAngle,wheelAngle+newTwist,wheelAngle+newTwist);
   			turnWheels(wheelAngle,wheelAngle,wheelAngle+newTwist,wheelAngle+newTwist);
    		}
    	}
    	else if (wheelAngle >= 45) {
    		if (power > 0) {
 //   			adjustAllWheels(wheelAngle,wheelAngle+newTwist,wheelAngle,wheelAngle+newTwist);
    			turnWheels(wheelAngle,wheelAngle+newTwist,wheelAngle,wheelAngle+newTwist);
    		}
    		else {
 //   			adjustAllWheels(wheelAngle+newTwist,wheelAngle,wheelAngle+newTwist,wheelAngle);
    			adjustAllWheels(wheelAngle+newTwist,wheelAngle,wheelAngle+newTwist,wheelAngle);
    		}
    	}
    	else if (wheelAngle <= -45) {
    		if (power > 0) {
//    			adjustAllWheels(wheelAngle+newTwist,wheelAngle,wheelAngle+newTwist,wheelAngle);
    			turnWheels(wheelAngle+newTwist,wheelAngle,wheelAngle+newTwist,wheelAngle);
    		}
    		else {
//    			adjustAllWheels(wheelAngle,wheelAngle+newTwist,wheelAngle,wheelAngle+newTwist);
    			turnWheels(wheelAngle,wheelAngle+newTwist,wheelAngle,wheelAngle+newTwist);
    		}
    	}
    	
    	driveAllWheels(power,power,power,power);
    	
    	printAngle = wheelAngle;
    }
    
    void slowPOVDrive(int anglePOV) {
    	if (anglePOV > -40 && anglePOV < 43) {
    		anglePOV = 0; 
    		saveX = 0;
    		saveY = 0.7;
    	}
    	else if (anglePOV > 43 && anglePOV < 140) {
    		anglePOV = 90;
    		saveX = 0.7;
    		saveY = 0;
    	}
    	else if (anglePOV > 140 && anglePOV < 230) {
    		anglePOV = 180;
    		saveX = 0;
    		saveY = -0.7;
    	}
    	else {
    		anglePOV = -90;
    		saveX = -0.7;
    		saveY = 0;
    	}

    	boolean changeDirection = bestPadAngle(anglePOV);
    	if (changeDirection) {
    		if (anglePOV >= 0) anglePOV = anglePOV - 180;
    		else anglePOV = anglePOV + 180;		
    	}
    	adjustAllWheels(anglePOV,anglePOV,anglePOV,anglePOV);
    	if (setLF && setRF && setLR && setRR) allSet = true; 
    	if (allSet) 
    	{
    		if (changeDirection) {
    			driveAllWheels(-0.4,-0.4,-0.4,-0.4);
    		}
    		else driveAllWheels(0.4,0.4,0.4,0.4);
    	}
    	else driveAllWheels(0,0,0,0);
    }

    void turnInPlace(double power) {

    	saveX = 0;
    	saveY = 0.7;

    	adjustAllWheels(45.,-45.,-45.,45.);
    	if (deltaLF<4 && deltaRF<4 && deltaLR<4 && deltaRR<4) allSet = true;
    	if (!allSet) {
    		driveAllWheels(0,0,0,0);
    	}
    	else {
    		if (Math.abs(power) > .01)  {
    			double ratioP = Math.abs(driveLF.get()/power);
    			if (ratioP < 1) ratioP = ratioP + 0.1;
    			if (ratioP > 1) ratioP = 1;
    			power = ratioP * power;
    		}
    			driveAllWheels(power,-power,power,-power);
    	}

    }
    
    void alternateTIP(double power) {
    	saveX = 0;
    	saveY = 0.7;

    	adjustAllWheels(-135.,-45.,135.,45.);
    	if (deltaLF<4 && deltaRF<4 && deltaLR<4 && deltaRR<4) allSet = true;
    	if (!allSet) {
    		driveAllWheels(0,0,0,0);
    	}
    	else {
    		if (Math.abs(power) > .01)  {
    			double ratioP = Math.abs(driveLF.get()/power);
    			if (ratioP < 1) ratioP = ratioP + 0.1;
    			if (ratioP > 1) ratioP = 1;
    			power = ratioP * power;
    		}
    			driveAllWheels(power,-power,power,power);
    	}
    	
    }
    
    void slightTurnRight() {
    	saveX = 0;
    	saveY = 0.7;
    	adjustAllWheels(60,60,0,0);
    	if (setLF && setRF && setLR && setRR) allSet = true;
    	if (allSet) {
    		driveAllWheels(0.5,0.5,0.3,0.33);
    	}
    }

    void slightTurnLeft()  {
    	saveX = 0;
    	saveY = 0.7;
    	adjustAllWheels(-60,-60,0,0);
    	if (setLF && setRF && setLR && setRR) allSet = true;
    	if (allSet) {
    		driveAllWheels(0.5,0.5,0.3,0.3);
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

    void controlLifter()  {
    	double power = funBox.getY();
    	if (power > 0 && limitBottom.get()) power = 0;
    	//   	else if (power > 0 && limitTop.get()) power = 0;
    	//   	power = 0.5 * power;
    	if (funBox.getZ() <= 0.5) {
    		lifterOne.set(power);
    		lifterTwo.set(power);

    		if (funBox.getRawButton(1)) {
    			armLeft.set(false);
    			armRight.set(false);
    		}
    		else if (funBox.getRawButton(2)) {
    			armLeft.set(false);
    			armRight.set(true);
    		}
    		else if (funBox.getRawButton(3)) {
    			armLeft.set(true);
    			armRight.set(false);
    		}
    		else if (funBox.getRawButton(4)) {
    			armLeft.set(true);
    			armRight.set(true);
    		}
    	}
    }
    void canTainerControls() {
    	if (canBox.getRawButton(5)) canRight.set(Value.kForward);
    	if (canBox.getRawButton(6))  canLeft.set(Value.kForward);
    	if (canBox.getRawButton(8)) {
    		canRight.set(Value.kReverse);
    		canLeft.set(Value.kReverse);
    	}
    	if (canBox.getRawButton(7))  finger.set(true);
    	else finger.set(false);
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
    	turnLF.set(powerLF);
    	lastDeltaLF = deltaLF;
    	
    }*/
    
    void adjustLF(double newAngle) {
    	
    	double oldLF = encoderLF.getRaw()*360.0/ENCODERcount;
    	if (oldLF > 180) oldLF = oldLF - 360;
    	else if (oldLF < -180) oldLF = oldLF + 360;
    	if (newAngle > 180) newAngle = newAngle - 360;
    	else if (newAngle < -180) newAngle = newAngle + 360;
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
    	turnLF.set(powerLF);
  //  	lastDirLF = dirLF;
    }

    void adjustRF(double newAngle) {
    	double oldRF = encoderRF.getRaw()*360.0/ENCODERcount;
    	if (oldRF > 180) oldRF = oldRF - 360;
    	else if (oldRF < -180) oldRF = oldRF + 360;
    	if (newAngle > 180) newAngle = newAngle - 360;
    	else if (newAngle < -180) newAngle = newAngle + 360;
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
    	turnRF.set(powerRF);
 //   	lastDirRF = dirRF;
    }
    
    void adjustLR(double newAngle) {
    	double oldLR = encoderLR.getRaw()*360.0/ENCODERcount;
    	if (oldLR > 180) oldLR = oldLR - 360;
    	else if (oldLR < -180) oldLR = oldLR + 360;
    	if (newAngle > 180) newAngle = newAngle - 360;
    	else if (newAngle < -180) newAngle = newAngle + 360;
    	dirLR = getDirection(oldLR,newAngle);
    	deltaLR = Math.abs(newAngle - oldLR);
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
    	turnLR.set(powerLR);
//    	lastDirLR = dirLR;
    }
    
    void adjustRR(double newAngle) {
    	double oldRR = encoderRR.getRaw()*360.0/ENCODERcount;
    	if (oldRR > 180) oldRR = oldRR - 360;
    	else if (oldRR < -180) oldRR = oldRR + 360;
    	if (newAngle > 180) newAngle = newAngle - 360;
    	else if (newAngle < -180) newAngle = newAngle + 360;
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
    	turnRR.set(powerRR);
//    	lastDirRR = dirRR;
    }
    
    void adjustAllWheels(double newAngleLF,double newAngleRF,double newAngleLR,double newAngleRR) {
    	adjustLF(newAngleLF);
    	adjustRF(newAngleRF);
    	adjustLR(newAngleLR);
    	adjustRR(newAngleRR);
    }
    
    void driveAllWheels(double powerLF,double powerRF,double powerLR,double powerRR) {
    	driveLF.set(powerLF);
    	driveRF.set(powerRF);
    	driveLR.set(powerLR);
    	driveRR.set(powerRR);
    }
    
    void stopAllWheels() {
    	turnLF.set(0);
    	turnRF.set(0);
    	turnLR.set(0);
    	turnRR.set(0);
    driveAllWheels(0,0,0,0);
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
//    	slowAngle = slowAngle*4.;
//    	gain = gain * .004;
    	slowAngle = 12;
    	gain = .012;

    	if (error <= setPoint) power = 0;
    	else if (error <= slowAngle) power = 0.2;  	
    	else {
    	 power = 0.2 + gain*(error - slowAngle);   	
    	}
    	if (power > 0.45) power = 0.45;
    	
    	return power;

    
    }
    
    double computeDrivePower(double y,double x) {
    	double power = Math.sqrt(x*x + y*y);
    	return power;
    }
    
    double getMaxPower(double a,double b,double c,double d) {
    	double max = Math.abs(a);
    	
    	if (Math.abs(b) > max) max = Math.abs(b);
    	if (Math.abs(c) > max) max = Math.abs(c);
    	if (Math.abs(d) > max) max = Math.abs(d);
    	return max;
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
    
    boolean chooseBestAngle(double y,double x) {
     	double newAngle;
     	boolean angleReverse;
     	double currentAngle = getCurrentAngle();
    	newAngle = 180*Math.atan2(y, x)/Math.PI;
    	newAngle = -newAngle + 90;
    	if (newAngle >= 180) newAngle = newAngle - 360;
    	if (newAngle - currentAngle > 110) {
    		newAngle = newAngle - 180;
    		if (newAngle < -180) newAngle = newAngle + 360;
    		angleReverse = true;
    	}
    	else if (newAngle - currentAngle < -110) {
    		newAngle = newAngle + 180;
    		if (newAngle >= 180) newAngle = newAngle - 360;
    		angleReverse = true;
    	}
    	else angleReverse = false;
    	return angleReverse;
    	
    }
    
    boolean bestPadAngle(double angle) {
    	boolean angleReverse;
     	double currentAngle = getCurrentAngle();
     	if (angle - currentAngle > 110) angleReverse = true;
     	else if (angle - currentAngle < -110) angleReverse = true;
     	else angleReverse = false;
     	return angleReverse;
    }
    
    double getCurrentAngle()  {
    	double oldLF = encoderLF.getRaw()*360.0/ENCODERcount;
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
 //   	overshootLF = false;
 //   	overshootRF = false;
 //   	overshootLR = false;
//   	overshootRR = false;
//    	sumLF = 5;
//    	sumRF = 5;
 //   	sumLR = 5;
//    	sumRR = 5;
    }
    
    void encoderRevolutionCheck() {
    	int pulsesLF = encoderLF.getRaw();
    	int pulsesRF = encoderRF.getRaw();
    	int pulsesLR = encoderLR.getRaw();
    	int pulsesRR = encoderRR.getRaw();
    	if (pulsesLF > ENCODERcount || pulsesLF < -ENCODERcount) encoderLF.reset();
    	if (pulsesRF > ENCODERcount || pulsesRF < -ENCODERcount) encoderRF.reset();
    	if (pulsesLR > ENCODERcount || pulsesLR < -ENCODERcount) encoderLR.reset();
    	if (pulsesRR > ENCODERcount || pulsesRR < -ENCODERcount) encoderRR.reset();
    	
    }
    
    double limit(double value) {
    	if (value > 1) return 1;
    	else if (value < -1) return -1;
    	else return value;
    }
    
    void autoStraight(double wheelAngle,double power) {
    	
    	double twist;
    	
    	double bearing = moeGyro.getAngle();
    	
    	if (bearing < lowBearing) lowBearing = bearing;
    	if (bearing > highBearing) highBearing = bearing;
    	sumBearing = sumBearing + bearing;
    	avgBearing = sumBearing/loopCount;
    
    	double proportional = 2. * SmartDashboard.getNumber("DB/Slider 0", 0.0);   //1 for now
    	double startSum = SmartDashboard.getNumber("DB/Slider 1", 0.0);      //try 1 to start
    	double maxTwist = 4 * SmartDashboard.getNumber("DB/Slider 2", 0.0);   //4 for now
    	
 //   	double proportional = 2.5;
 //   	double maxTwist = 15;
    	
    	if (bearing > 0.75 && straightSum < 0.1 && straightSum > -0.1) straightSum = -startSum;
    	else if (bearing < -0.75 && straightSum < 0.1 && straightSum > -0.1) straightSum = startSum;
   /* 	
    	if (bearing > 0.8 && bearing < 8) {
    		straightSum = straightSum - addSum;
    	}
    	else if (bearing < -0.8 && bearing > -8) {
    		straightSum = straightSum + addSum;
    	}
    */
    	twist = -proportional*bearing;

 //   	if (twist > maxTwist) twist = maxTwist;
  //  	else if (twist < -maxTwist) twist = -maxTwist;
  //  	double newTwist = twist;
    	double newTwist = straightSum + twist;
    	// 	if (newTwist > 3) newTwist = 3;
    	// 	else if (newTwist < -3) newTwist = -3;

    	//   	newTwist = straightSum + twist;

     	
    	if (wheelAngle > -45 && wheelAngle < 45) {
    		if (power > 0) {
    			adjustAllWheels(wheelAngle+newTwist,wheelAngle+newTwist,wheelAngle,wheelAngle);
    			
    		}
    		else {
    			adjustAllWheels(wheelAngle,wheelAngle,wheelAngle+newTwist,wheelAngle+newTwist);
    		}
    	}
    	else if (wheelAngle >= 45) {
    		if (power > 0) {
    			adjustAllWheels(wheelAngle,wheelAngle+newTwist,wheelAngle,wheelAngle+newTwist);
    		}
    		else {
    			adjustAllWheels(wheelAngle+newTwist,wheelAngle,wheelAngle+newTwist,wheelAngle);
    		}
    	}
    	else if (wheelAngle <= -45) {
    		if (power > 0) {
    			adjustAllWheels(wheelAngle+newTwist,wheelAngle,wheelAngle+newTwist,wheelAngle);
    		}
    		else {
    			adjustAllWheels(wheelAngle,wheelAngle+newTwist,wheelAngle,wheelAngle+newTwist);
    		}
    	}
    	
    }
    
    void turnWheels(double newLF,double newRF,double newLR,double newRR) {
    	double powerLF;
    	double powerRF;
    	double powerLR;
    	double powerRR;
    	double setPower;
    	
    	setPower = 0.3;
    	
    	double oldLF = encoderLF.getRaw()*360.0/ENCODERcount;
    	if (oldLF > 180) oldLF = oldLF - 360;
    	else if (oldLF < -180) oldLF = oldLF + 360;
    	if (newLF > 180) newLF = newLF - 360;
    	else if (newLF < -180) newLF = newLF + 360;
    	dirLF = getDirection(oldLF,newLF);
    	deltaLF = Math.abs(newLF - oldLF);
    	if (deltaLF <= setPoint) powerLF = 0;
    	else if (dirLF) powerLF = setPower;
    	else powerLF = -setPower;
    	turnLF.set(powerLF);
    	
    	double oldRF = encoderRF.getRaw()*360.0/ENCODERcount;
    	if (oldRF > 180) oldRF = oldRF - 360;
    	else if (oldRF < -180) oldRF = oldRF + 360;
    	if (newRF > 180) newRF = newRF - 360;
    	else if (newRF < -180) newRF = newRF + 360;
    	dirRF = getDirection(oldRF,newRF);
    	deltaRF = Math.abs(newRF - oldRF);
    	if (deltaRF <= setPoint) powerRF = 0;
    	else if (dirRF) powerRF = setPower;
    	else powerRF = -setPower;
    	turnRF.set(powerRF);
    	
    	double oldLR = encoderLR.getRaw()*360.0/ENCODERcount;
    	if (oldLR > 180) oldLR = oldLR - 360;
    	else if (oldLR < -180) oldLR = oldLR + 360;
    	if (newLR > 180) newLR = newLR - 360;
    	else if (newLR < -180) newLR = newLR + 360;
    	dirLR = getDirection(oldLR,newLR);
    	deltaLR = Math.abs(newLR - oldLR);
    	if (deltaLR <= setPoint) powerLR = 0;
    	else if (dirLR) powerLR = setPower;
    	else powerLR = -setPower;
    	turnLR.set(powerLR);
    	
    	double oldRR = encoderRR.getRaw()*360.0/ENCODERcount;
    	if (oldRR > 180) oldRR = oldRR - 360;
    	else if (oldRR < -180) oldRR = oldRR + 360;
    	if (newRR > 180) newRR = newRR - 360;
    	else if (newRR < -180) newRR = newRR + 360;
    	dirRR = getDirection(oldRR,newRR);
    	deltaRR = Math.abs(newRR - oldRR);
    	if (deltaRR <= setPoint) powerRR = 0;
    	else if (dirRR) powerRR = setPower;
    	else powerRR = -setPower;
    	turnRR.set(powerRR);
    }
    
    void raiseLifter(int height) {
    	if (liftEncoder.getRaw() > height) {
    		lifterOne.set(0);
    		lifterTwo.set(0);
    	}
    	else {
    		lifterOne.set(-1.0);
    		lifterTwo.set(-1.0);
    	}
    }
    
    void lowerLifter(int height) {
    	if (liftEncoder.getRaw() < height) {
    		lifterOne.set(0);
    		lifterTwo.set(0);
    	}
    	else {
    		lifterOne.set(0.75);
    		lifterTwo.set(0.75);
    	}
    }
    
    void testRoutine() {
    	double x = xBox.getX();
    	double y = -xBox.getRawAxis(5);
    	
    	if (x<0.1 && x>-0.1) x = 0;
    	if (y<0.1 && y>-0.1) y = 0;
    	
    	turnLF.set(x);
    	turnRF.set(x);;
    	turnLR.set(x);
    	turnRR.set(x);
    	
    	driveLF.set(y);
    	driveRF.set(y);
    	driveLR.set(y);
    	driveRR.set(y);
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
    		printAngle = wheelAngle;
    		adjustAllWheels(wheelAngle,wheelAngle,wheelAngle,wheelAngle);

    		if (setLF && setRF && setLR && setRR) allSet = true;
    		if (allSet) {
    			countTime++;
    			if (countTime < 50)  driveAllWheels(0.4,0.4,0.4,0.4);
    			else driveAllWheels(0,0,0,0);
    		}
    		else driveAllWheels(0,0,0,0);

    	}
    	else driveAllWheels(0,0,0,0);

    }
    void autoStackRoutine() {
    	double power;
//    	double targetDist;
    	switch(autoStep) {
    	case 1: 
    		adjustAllWheels(-90,-90,-90,-90);
//    		raiseLifter(80);
    		if (loopCount > 50) {
 //   		if (liftEncoder.getRaw() > 10)  {
 //   		if (setLF && setRF && setLR && setRR) {
    			autoStep = 2;
    			straightSum = 0;
    			loopCount = 0;
    			distEncoder.reset();
    			lowBearing = 0;
    			highBearing = 0;
    			avgBearing = 0;
    			sumBearing = 0;
    		}
    		break;
    	case 2:
    		power = 0.1 + loopCount*.05;
    		if (power > 0.4) power = 0.4; 
    		if (distEncoder.getRaw() > 4000) {
    			//   		if (distEncoder.getRaw() > 4550) {
    			if (!leftEye.get() && !rightEye.get()) {
    				autoStep = 3;
    				driveAllWheels(0,0,0,0);
    				SmartDashboard.putNumber("lowB", lowBearing);
    				SmartDashboard.putNumber("highB", highBearing);
    				SmartDashboard.putNumber("avgB", avgBearing);
    			}
    		}
    		else {
    			autoStraight(-90,power);
    			driveAllWheels(0.4,0.4,0.4,0.4);
  //  			raiseLifter(80);
    		}
    		break;
    	case 3:
    		adjustAllWheels(0,0,0,0);
 //   		lowerLifter(0);
    		targetDist = distIR.getAverageVoltage()*(-.148) + 2.39;
    		targetDist = targetDist * 47;
    		if (setLF && setRF && setLR && setRR) {
    			autoStep = 4;
    			distEncoder.reset();
    		}
    		break;
    	case 4:
    		adjustAllWheels(0,0,0,0);
    		driveAllWheels(0.4,0.4,0.4,0.4);
    		if (distEncoder.getRaw() > targetDist) {
    			autoStep = 5;
    			driveAllWheels(0,0,0,0);
    			loopCount = 0;
    		}
    		break;
    	case 5:
    		adjustAllWheels(-93,-93,-93,-93);
 //   		raiseLifter(500);
    		if (loopCount > 50) {
//    		if (liftEncoder.getRaw() < 0) {
  //  		if (setLF && setRF && setLR && setRR) {
    			autoStep = 6;  		
    		}
    		break;
    	case 6:
    		adjustAllWheels(-93,-93,-93,-93);
 //   		raiseLifter(80);
    		if (liftEncoder.getRaw() > 10) {
    			autoStep = 7;
    			straightSum = 0;
    			distEncoder.reset();
    			SmartDashboard.putNumber("gyro", moeGyro.getAngle());
    			loopCount = 0;
    			lowBearing = 0;
    			highBearing = 0;
    			avgBearing = 0;
    			sumBearing = 0;
    		}
    	case 7:
    		power = 0.1 + loopCount*.05;
    		if (power > 0.4) power = 0.4;    		
    		if (distEncoder.getRaw() > 4550) {
    			autoStep = 8;
    			driveAllWheels(0,0,0,0);
    			SmartDashboard.putNumber("lowB", lowBearing);
    			SmartDashboard.putNumber("highB", highBearing);
    			SmartDashboard.putNumber("avgB", avgBearing);
    		}
    		else {
    			autoStraight(-90,power);
    			driveAllWheels(0.4,0.4,0.4,0.4);
    		}
    		break;
    	case 8:
    		adjustAllWheels(0,0,0,0);
  //   		lowerLifter(0);
    		targetDist = distIR.getAverageVoltage()*(-.148) + 2.39;
    		targetDist = targetDist * 47;
    		if (setLF && setRF && setLR && setRR) {
    			autoStep = 9;
    			distEncoder.reset();
    		}
    		break;
    	case 9:
    		adjustAllWheels(0,0,0,0);
    		driveAllWheels(0.4,0.4,0.4,0.4);
    		if (distEncoder.getRaw() > targetDist) {
    			autoStep = 10;
    			driveAllWheels(0,0,0,0);
    			loopCount = 0;
    		}
    		break;
    	case 10:
    		adjustAllWheels(-35,-90,-180,-135);
 //   		raiseLifter(500);
    		if (loopCount > 50) {
//    		if (liftEncoder.getRaw() < 0) {
  //  		if (setLF && setRF && setLR && setRR) {
    			autoStep = 11;  		
    		}
    		break;
    	case 11:
    		if (moeGyro.getAngle()< -70) {
    			autoStep = 12;
    			distEncoder.reset();
    		}
    		else {
    			driveAllWheels(0.6,0.4,0.4,0);
    		}
    		break;
    	case 12:
    		adjustAllWheels(-90,-90,-90,-90);
    		driveAllWheels(0.4,0.4,0.4,0.4);
    		if (distEncoder.getRaw() > 2000) {
    			autoStep = 13;
    			driveAllWheels(0,0,0,0);
    		}
    		break;
    	case 13:
    		adjustAllWheels(-90,-90,-90,-90);
    		driveAllWheels(0,0,0,0);
    		

    	default:
    			adjustAllWheels(0,0,0,0);
        		driveAllWheels(0,0,0,0);
    			
    	}    	
    	
    }
    
    void moveStackToPlatform() {
    
    	switch (stackStep) {
    	case 1:
    		adjustAllWheels(0,0,0,0);
    		if (setLF && setRF && setLR && setRR) {
    			stackStep = 2;
    			distEncoder.reset();
    		}
    		break;
    	case 2:
    		adjustAllWheels(0,0,0,0);
    		if (distEncoder.getRaw() < -500) {
    			stackStep = 3;
    			moeGyro.reset();
    			driveAllWheels(0,0,0,0);
    			allSet = false;
    		}
    		else driveAllWheels(-0.4,-0.4,-0.4,-0.4);
    		break;
    	case 3:
    		if (moeGyro.getAngle() < -125) {
    			stackStep = 4;
    			driveAllWheels(0,0,0,0);
    		}
  //  		else turnInPlace(-0.4);
    		else alternateTIP(0.45);
    		break;
    	case 4:
    		adjustAllWheels(0,0,0,0);
    		if (setLF && setRF && setLR && setRR) {
    			stackStep = 5;
    			distEncoder.reset();
    		}
    		break;
    	case 5:
    		adjustAllWheels(0,0,0,0);
    		if (distEncoder.getRaw() > 500) {
    			stackStep = 6;
    			driveAllWheels(0,0,0,0);
    		}
    		else driveAllWheels(0.4,0.4,0.4,0.4);
    		break;
    	case 6:
    		driveAllWheels(0,0,0,0);
    		adjustAllWheels(0,0,0,0);
    		break;
    	default:
    		driveAllWheels(0,0,0,0);
    		adjustAllWheels(0,0,0,0);

    	}
    }
    
    void lineUpAtFeeder() {
    	boolean changeDirection = bestPadAngle(0);
    	if (changeDirection) 
    		adjustAllWheels(180,180,180,180);
    	else adjustAllWheels(0,0,0,0);

    	switch(stackStep) {
    	case 1:
    		driveAllWheels(0,0,0,0);
 //   		armLeft.set(true);
//    		armRight.set(true);
    		if (liftEncoder.getRaw() < toteOne) {
    			lifterOne.set(0);
    			lifterTwo.set(0);
      			stackStep = 2;
    			distEncoder.reset();
    		}
    		else {
    			lifterOne.set(0.75);
    			lifterTwo.set(0.75);
    		}
    		
    		break;
    	case 2:
    		armLeft.set(true);
    		armRight.set(true);
    		if (changeDirection) driveAllWheels(0.35,0.35,0.35,0.35);
    		else driveAllWheels(-0.35,-0.35,-0.35,-0.35);
    		if (distEncoder.getRaw() < -200) {
    			stackStep = 3;
    			driveAllWheels(0,0,0,0);
    		}
    		break;
    	case 3:
    		if (liftEncoder.getRaw() > toteTwo) {
    			lifterOne.set(0);
    			lifterTwo.set(0);
    			stackStep = 4;
    		}
    		else {
    			lifterOne.set(-1.0);
    			lifterTwo.set(-1.0);
    		}
    		break;
    	case 4:
    		if (changeDirection) driveAllWheels(-0.35,-0.35,-0.35,-0.35);
    		else driveAllWheels(0.35,0.35,0.35,0.35);
    		if (distEncoder.getRaw()> -100) {
    			stackStep = 5;
    			driveAllWheels(0,0,0,0);
    		}
    		break;
    	case 5:
    		driveAllWheels(0,0,0,0);
    		break;
    	default:
    			driveAllWheels(0,0,0,0);
    		
    	}
    }
    
    void stackAtFeeder() {
    	stopAllWheels();
    	loopCount++;
    	switch(stackStep) {
    	case 1:
    		armLeft.set(false);
    		armRight.set(true);
    		if (loopCount > 25) {
    			stackStep = 2;
    		}
    		
    		break;
    	case 2:
    		raiseLifter(toteFour);
    		if (lifterOne.get()> -0.05) {
    			stackStep = 3;
    		}
    		break;
    	case 3:
    		if (funBox.getZ() < -0.5){
    			stackStep = 4;
    		}
    		break;
    	case 4:
    		lowerLifter(toteThree);
    		if (liftEncoder.getRaw() < toteThree) {
    			stackStep = 5;
    		}
    		break;
    	case 5:
    		armLeft.set(true);
    		armRight.set(true);
    		lowerLifter(toteTwo);
    		if (lifterOne.get() < 0.05) {
    			stackStep = 1;
    			loopCount = 0;
    		}
    		break;
    		default:
    			lifterOne.set(0);
    			lifterTwo.set(0);
    		
    		
    	}
    }
    
    void carModeBackUp() {
    	switch (stackStep) {
    	case 1:
    		adjustAllWheels(-60,-60,0,0);
    		if (setLF && setRF && setLR && setRR) {
    			stackStep = 2;
    			moeGyro.reset();
    		}
    		break;
    	case 2:
    		adjustAllWheels(-60,-60,0,0);
    		if (moeGyro.getAngle() > 90) {
    			stackStep = 3;
    			driveAllWheels(0,0,0,0);
    		}
    		driveAllWheels(-.4,-.4,-.4,-.4);
    		break;
    	case 3:
    		adjustAllWheels(60,60,0,0);
    		if (setLF && setRF && setLR && setRR) {
    			stackStep = 4;
    		}
    		break;
    	case 4:
    		adjustAllWheels(60,60,0,0);
    		if (moeGyro.getAngle() > 130)  {
    			stackStep = 5;
    			driveAllWheels(0,0,0,0);
    		}
    		else driveAllWheels(0.4,0.4,0.4,0.4);
    		break;
    	case 5:
    		adjustAllWheels(0,0,0,0);
    		driveAllWheels(0,0,0,0);
    		break;
    	default:
    		adjustAllWheels(0,0,0,0);
    		driveAllWheels(0,0,0,0);

    	}
    }


}
