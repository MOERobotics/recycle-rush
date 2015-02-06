package org.usfirst.frc.team365.robot;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Direction;
import edu.wpi.first.wpilibj.Relay.Value;

/**
 * Ben Hylak
 *  
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	Joystick stick;
	//int autoLoopCounter;
	Relay relay;
	CANTalon talon;
	
	
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
    	
    	talon = new CANTalon(1);
    	stick = new Joystick(0);
    	relay = new Relay(0);
    	
    }
    
    /**
     * This function is run once each time the robot enters autonomous mode
     */
    public void autonomousInit() {
    	//autoLoopCounter = 0;
    	talon.set(0);
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
    	talon.set(1);
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
    	relay.setDirection(Direction.kForward);
    	relay.set(Value.kOn);
        talon.set(stick.getY());
        
        int Xloc = 0;
        int Yloc = 1;
        int angle = 2;
        
        int wheel1=0;
        int wheel2=1;
        int wheel3=2;
        int wheel4=3;
        
        double[][] wheelVals = new double[4][3]; 
        
        double wheel1Y = wheelVals[wheel1][Yloc];
        double wheel1Angle = wheelVals[wheel1][angle];
        
        
        
        
        
        
        
        //wheelVal
        //to access wheel 1 X use:
        // wheelVals[0][0] (remember arrays are 0 indexed)
        // for wheel 1 Y use wheelVals[0][1] 
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    	//LiveWindow.run();
    }
    
}
