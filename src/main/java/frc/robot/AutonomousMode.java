package frc.robot;

import java.lang.invoke.MethodHandles;

import frc.drivetrain.Drivetrain;


public class AutonomousMode implements ModeTransition
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
  

    // *** CLASS CONSTRUCTOR ***
    public AutonomousMode()
    {
        System.out.println(fullClassName + " : Constructor Started");

        System.out.println(fullClassName + ": Constructor Finished");
    }

    /**
     * This method runs one time before the periodic() method.
     */
    public void init()
    {
       

        if(DRIVETRAIN != null)
        {
            DRIVETRAIN.resetOdometry();
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
