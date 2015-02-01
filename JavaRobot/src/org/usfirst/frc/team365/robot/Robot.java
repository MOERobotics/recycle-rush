package org.usfirst.frc.team365.robot;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Direction;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {

    CANTalon[] talons = new CANTalon[16];
    String[] talon_name = new String[16];
    DriverStation ds;

    public Robot() {
        ds = DriverStation.getInstance();
        for (int i = 0; i < 16; i++) {
            try {
                talons[i] = new CANTalon(i);
                talon_name[i] = talons[i].getDescription();
            } catch (RuntimeException e) {
                talons[i]=null;
                talon_name[i] = e.toString();
            }
            SmartDashboard.putString("talon_name"+i, talon_name[i]);
        }
    }
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {


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
        for(int i = 0; i < 16; i++) {
            talons[i].set(0);
            SmartDashboard.putNumber("Talons"+i,0.0);
        }
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
        for(int i = 0; i < 16; i++) if (talons[i] != null) {
            talons[i].set(
                limit(
                    (float)SmartDashboard.getNumber("Talons"+i)
                )
            );
        }
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    	//LiveWindow.run();
    }

    public float limit(float in, float max) {
        return (in > max)
                ? max
                : (in < -max)
                ? -max
                : in;
    }
    public float limit(float in) {return limit(in,1f);}
    
}
