package frc.robot;

import java.lang.invoke.MethodHandles;

import frc.drivetrain.Drivetrain;
import frc.robot.Robot.RobotState;


public class DisabledMode implements ModeTransition
{
    private static final String fullClassName = MethodHandles.lookup().lookupClass().getCanonicalName();

    // *** STATIC INITIALIZATION BLOCK ***
    // This block of code is run first when the class is loaded
    static
    {
        System.out.println("Loading: " + fullClassName);
    }

    // *** CLASS & INSTANCE VARIABLES ***
   
    private static final Drivetrain DRIVETRAIN = RobotContainer.DRIVETRAIN;
    
    private RobotState robotState;

    // *** CLASS CONSTRUCTOR ***
    public DisabledMode()
    {
        System.out.println(fullClassName + " : Constructor Started");

        System.out.println(fullClassName + ": Constructor Finished");
    }

    /**
     * This method runs one time before the periodic() method.
     */
    public void init()
    {
        robotState = Robot.getRobotState();

        if (DRIVETRAIN != null)
        {
            DRIVETRAIN.resetEncoders();
        }
    }

    /**
     * This method runs periodically (every 20ms).
     */
    public void periodic()
    {
       
    }

    /**
     * This method runs one time after the periodic() method.
     */
    public void exit()
    {
        
    }
}
